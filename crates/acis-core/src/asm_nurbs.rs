//! Explicit ASM 22700 / embedded 22601 splines with a bounded default trailer.
//!
//! This is an observed profile, not a general procedural subtype interpreter.
//! Support surfaces, pcurves, subtype references, reversal, non-default trailer
//! fields and nonzero fitting tolerances are deliberately not admitted.
use crate::nurbs::knot_vector;
use crate::{
    AcisValue, BSplineCurveEntity, BSplineSurfaceEntity, Entity, ParameterRange, RawEntity, Vec3,
};

pub(crate) struct Reader<'a> {
    pub(crate) values: &'a [AcisValue],
    pub(crate) pos: usize,
}
impl Reader<'_> {
    pub(crate) fn take(&mut self) -> Result<&AcisValue, String> {
        let v = self
            .values
            .get(self.pos)
            .ok_or("truncated explicit ASM spline")?;
        self.pos += 1;
        Ok(v)
    }
    pub(crate) fn byte(&mut self, expected: u8) -> Result<(), String> {
        if matches!(self.take()?, AcisValue::Bytes(v) if v == &[expected]) {
            Ok(())
        } else {
            Err(format!("explicit ASM spline expected tag {expected}"))
        }
    }
    pub(crate) fn word(&mut self, expected: &str) -> Result<(), String> {
        if matches!(self.take()?, AcisValue::String(v) if v == expected) {
            Ok(())
        } else {
            Err(format!("explicit ASM spline expected {expected}"))
        }
    }
    pub(crate) fn integer(&mut self) -> Result<usize, String> {
        match self.take()? {
            AcisValue::Integer(v) => v
                .to_string()
                .parse()
                .map_err(|_| "invalid ASM spline integer".into()),
            _ => Err("expected ASM spline integer".into()),
        }
    }
    pub(crate) fn int(&mut self, expected: usize) -> Result<(), String> {
        if self.integer()? == expected {
            Ok(())
        } else {
            Err("unqualified ASM spline field".into())
        }
    }
    pub(crate) fn embedded_version(&mut self) -> Result<(), String> {
        if crate::qualified_embedded_version(self.integer()?) {
            Ok(())
        } else {
            Err("unqualified embedded subtype version".into())
        }
    }
    pub(crate) fn number(&mut self) -> Result<f64, String> {
        let v = match self.take()? {
            AcisValue::Float(v) => *v,
            AcisValue::Integer(v) => v
                .to_string()
                .parse()
                .map_err(|_| "invalid ASM spline number")?,
            _ => return Err("expected ASM spline number".into()),
        };
        if v.is_finite() {
            Ok(v)
        } else {
            Err("nonfinite ASM spline number".into())
        }
    }
    pub(crate) fn scalar(&mut self, expected: f64) -> Result<(), String> {
        if self.number()? == expected {
            Ok(())
        } else {
            Err("unqualified ASM spline scalar".into())
        }
    }
    pub(crate) fn bound(&mut self) -> Result<Option<f64>, String> {
        match self.take()? {
            AcisValue::Bytes(v) if v == &[0x0b] => Ok(None),
            AcisValue::Bytes(v) if v == &[0x0a] => Ok(Some(self.number()?)),
            _ => Err("invalid ASM spline bound".into()),
        }
    }
    pub(crate) fn range(&mut self) -> Result<Option<ParameterRange>, String> {
        Ok(Some(ParameterRange {
            lower: self.bound()?,
            upper: self.bound()?,
        }))
    }
    pub(crate) fn knots(
        &mut self,
        degree: usize,
        count: usize,
    ) -> Result<(Vec<f64>, Vec<usize>, usize), String> {
        if !(1..=16).contains(&degree) || !(2..=4096).contains(&count) {
            return Err("ASM spline knot/degree limit".into());
        }
        let mut knots = Vec::with_capacity(count);
        let mut mult = Vec::with_capacity(count);
        for _ in 0..count {
            knots.push(self.number()?);
            mult.push(self.integer()?);
        }
        if mult[0] != degree || mult[count - 1] != degree {
            return Err("unqualified ASM endpoint convention".into());
        }
        mult[0] += 1;
        mult[count - 1] += 1;
        let length = mult
            .iter()
            .try_fold(0usize, |a, b| a.checked_add(*b))
            .ok_or("knot overflow")?;
        let poles = length.checked_sub(degree + 1).ok_or("invalid pole count")?;
        knot_vector(degree, &knots, &mult, poles)?;
        Ok((knots, mult, poles))
    }
    pub(crate) fn poles(
        &mut self,
        count: usize,
        rational: bool,
    ) -> Result<(Vec<Vec3>, Vec<f64>), String> {
        if count > 1_000_000 {
            return Err("ASM spline pole limit".into());
        }
        // Check the remaining input before allocating untrusted counts.
        let width = if rational { 4 } else { 3 };
        if count > (self.values.len() - self.pos) / width {
            return Err("truncated ASM spline poles".into());
        }
        let mut poles = Vec::with_capacity(count);
        let mut weights = Vec::with_capacity(count);
        for _ in 0..count {
            poles.push((self.number()?, self.number()?, self.number()?).into());
            weights.push(if rational { self.number()? } else { 1. });
        }
        Ok((poles, weights))
    }
    fn discontinuities(&mut self, knots: &[f64]) -> Result<(), String> {
        // First/second/third derivative break lists. They carry no geometry: AutoCAD 22300 files list
        // only some interior knots, so any increasing subset of the interior knots is admitted.
        let interior = &knots[1..knots.len() - 1];
        for _ in 0..3 {
            let count = self.integer()?;
            if count > interior.len() {
                return Err("unqualified discontinuity list".into());
            }
            let mut last = f64::NEG_INFINITY;
            for _ in 0..count {
                let k = self.number()?;
                if k <= last || !interior.contains(&k) {
                    return Err("unqualified discontinuity list".into());
                }
                last = k;
            }
        }
        Ok(())
    }
    fn default_interval(&mut self) -> Result<(), String> {
        // Retained verbatim in RawEntity. No chart transformation is inferred
        // from this observed default trailer (F 1 F 0).
        self.byte(0x0a)?;
        self.scalar(1.)?;
        self.byte(0x0a)?;
        self.scalar(0.)
    }
}

pub(crate) fn decode(raw: &RawEntity, save_version: u32) -> Result<Option<Entity>, String> {
    if !crate::qualified_save_version(save_version) {
        return Ok(None);
    }
    let kind = match (raw.type_name.as_str(), raw.values.get(3)) {
        ("intcurve-curve", Some(AcisValue::String(s))) if s == "exact_int_cur" => 1,
        ("spline-surface", Some(AcisValue::String(s))) if s == "exact_spl_sur" => 2,
        _ => return Ok(None),
    };
    let mut r = Reader {
        values: &raw.values,
        pos: 0,
    };
    let pattern = match r.take()? {
        AcisValue::Reference(v) => *v,
        _ => return Err("invalid spline pattern".into()),
    };
    // The sense flag reverses the surface normal only; the UV chart is unchanged. Curves stay forward-only.
    let reversed = match r.take()? {
        AcisValue::Bytes(v) if v == &[0x0b] => false,
        AcisValue::Bytes(v) if v == &[0x0a] && kind == 2 => true,
        _ => return Err("explicit ASM spline expected tag 11".into()),
    };
    r.byte(0x0f)?;
    r.word(if kind == 1 {
        "exact_int_cur"
    } else {
        "exact_spl_sur"
    })?;
    r.embedded_version()?;
    r.int(0)?; // Embedded profile and full representation.
    let rational = match r.take()? {
        AcisValue::String(s) if s == "nubs" => false,
        AcisValue::String(s) if s == "nurbs" => true,
        _ => return Err("unqualified spline representation".into()),
    };
    let ud = r.integer()?;
    let entity = if kind == 1 {
        r.int(0)?; // Open closure.
        let n = r.integer()?;
        let (knots, multiplicities, count) = r.knots(ud, n)?;
        let (poles, weights) = r.poles(count, rational)?;
        r.scalar(0.)?; // Exact representation: no fitted approximation admitted.
        r.word("null_surface")?;
        r.word("null_surface")?;
        r.word("nullbs")?;
        r.word("nullbs")?;
        r.byte(0x0b)?;
        r.byte(0x0b)?; // Unbounded support interval.
        r.discontinuities(&knots)?;
        r.int(0)?;
        r.default_interval()?;
        r.int(0)?;
        r.int(0)?;
        r.byte(0x10)?;
        let curve = BSplineCurveEntity {
            raw: raw.clone(),
            pattern,
            degree: ud,
            knots,
            multiplicities,
            poles,
            weights,
            parameter_range: r.range()?,
            fit_tolerance: 0.,
        };
        curve.validate()?;
        Entity::BSplineCurve(curve)
    } else {
        let vd = r.integer()?;
        r.int(0)?;
        r.int(0)?; // Open/open.
                   // U/V singularity: 0 none, 1 a collapsed boundary (same poles, same evaluation).
        for _ in 0..2 {
            if r.integer()? > 1 {
                return Err("unqualified ASM spline field".into());
            }
        }
        let un = r.integer()?;
        let vn = r.integer()?;
        let (u_knots, u_multiplicities, u_count) = r.knots(ud, un)?;
        let (v_knots, v_multiplicities, v_count) = r.knots(vd, vn)?;
        let count = u_count
            .checked_mul(v_count)
            .ok_or("surface pole overflow")?;
        let (poles, weights) = r.poles(count, rational)?;
        r.scalar(0.)?;
        r.discontinuities(&u_knots)?;
        r.discontinuities(&v_knots)?;
        r.byte(0x0b)?;
        r.default_interval()?;
        r.default_interval()?;
        r.int(0)?;
        r.byte(0x10)?;
        let u_range = r.range()?;
        let v_range = r.range()?;
        // In this forward, explicit chart the saved bounds use the knot
        // parameters unchanged. validate() requires a nonempty subdomain;
        // the consumer must also keep every face trim inside these bounds.
        let surface = BSplineSurfaceEntity {
            raw: raw.clone(),
            pattern,
            u_degree: ud,
            v_degree: vd,
            u_knots,
            v_knots,
            u_multiplicities,
            v_multiplicities,
            u_count,
            v_count,
            poles,
            weights,
            reversed,
            u_range,
            v_range,
            fit_tolerance: 0.,
        };
        surface.validate()?;
        Entity::BSplineSurface(surface)
    };
    if r.pos != r.values.len() {
        return Err("unconsumed ASM spline trailer".into());
    }
    Ok(Some(entity))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn surface() -> RawEntity {
        use AcisValue::{Bytes as B, Float as F, Integer as I, String as S};
        let mut raw = RawEntity::new(7, "spline-surface");
        raw.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            B(vec![11]),
            B(vec![15]),
            S("exact_spl_sur".into()),
            I(22601.into()),
            I(0.into()),
            S("nurbs".into()),
            I(1.into()),
            I(1.into()),
            I(0.into()),
            I(0.into()),
            I(0.into()),
            I(0.into()),
            I(2.into()),
            I(2.into()),
        ];
        for _ in 0..2 {
            raw.values.extend([F(0.), I(1.into()), F(1.), I(1.into())]);
        }
        for (x, y, z, w) in [
            (0., 0., 0., 1.),
            (2., 0., 0., 2.),
            (0., 3., 0., 1.),
            (2., 3., 4., 2.),
        ] {
            raw.values.extend([F(x), F(y), F(z), F(w)]);
        }
        raw.values.push(F(0.));
        raw.values.extend((0..6).map(|_| I(0.into())));
        raw.values.push(B(vec![11]));
        for _ in 0..2 {
            raw.values.extend([B(vec![10]), F(1.), B(vec![10]), F(0.)]);
        }
        raw.values.extend([I(0.into()), B(vec![16])]);
        for bound in [0.2, 0.8, 0.25, 0.75] {
            raw.values.extend([B(vec![10]), F(bound)]);
        }
        raw
    }

    #[test]
    fn finite_explicit_chart_retains_source_and_rational_geometry() {
        let raw = surface();
        let Some(Entity::BSplineSurface(s)) = decode(&raw, 22700).unwrap() else {
            panic!()
        };
        assert_eq!(s.raw, raw);
        assert_eq!(
            s.u_range,
            Some(ParameterRange {
                lower: Some(0.2),
                upper: Some(0.8)
            })
        );
        assert_eq!(
            s.v_range,
            Some(ParameterRange {
                lower: Some(0.25),
                upper: Some(0.75)
            })
        );
        let p = s.evaluate(0.5, 0.5).unwrap();
        assert!((p.x - 4. / 3.).abs() < 1e-12);
        assert!((p.y - 1.5).abs() < 1e-12);
        assert!((p.z - 4. / 3.).abs() < 1e-12);
        assert!(decode(&raw, 22600).unwrap().is_none());
    }

    #[test]
    fn finite_chart_never_accepts_bad_bounds_or_extended_profiles() {
        let raw = surface();
        for length in 4..raw.values.len() {
            let mut changed = raw.clone();
            changed.values.truncate(length);
            assert!(decode(&changed, 22700).is_err());
        }
        for value in [-0.01, 0.8, 1.1, f64::NAN, f64::INFINITY] {
            let mut changed = raw.clone();
            let pos = changed.values.len() - 7;
            changed.values[pos] = AcisValue::Float(value);
            assert!(decode(&changed, 22700).is_err());
        }
        for (pos, value) in [
            (4, AcisValue::Integer(22602.into())),
            (9, AcisValue::Integer(1.into())),
            (11, AcisValue::Integer(2.into())),
        ] {
            let mut changed = raw.clone();
            changed.values[pos] = value;
            assert!(decode(&changed, 22700).is_err());
        }
        let mut changed = raw;
        changed.values.push(AcisValue::Integer(0.into()));
        assert!(decode(&changed, 22700).is_err());
    }

    fn cubic_curve(breaks: &[f64]) -> RawEntity {
        use AcisValue::{Bytes as B, Float as F, Integer as I, String as S};
        let mut raw = RawEntity::new(3, "intcurve-curve");
        raw.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            B(vec![11]),
            B(vec![15]),
            S("exact_int_cur".into()),
            I(23100.into()),
            I(0.into()),
            S("nubs".into()),
            I(3.into()),
            I(0.into()),
            I(5.into()),
        ];
        for (k, m) in [(0., 3), (0.25, 1), (0.5, 1), (0.75, 1), (1., 3)] {
            raw.values.extend([F(k), I(m.into())]);
        }
        for i in 0..7 {
            raw.values.extend([F(i as f64), F((i * i) as f64), F(0.)]);
        }
        raw.values.push(F(0.));
        for w in ["null_surface", "null_surface", "nullbs", "nullbs"] {
            raw.values.push(S(w.into()));
        }
        raw.values
            .extend([B(vec![11]), B(vec![11]), I(0.into()), I(0.into())]);
        raw.values.push(I(breaks.len().into()));
        raw.values.extend(breaks.iter().map(|b| F(*b)));
        raw.values
            .extend([I(0.into()), B(vec![10]), F(1.), B(vec![10]), F(0.)]);
        raw.values.extend([I(0.into()), I(0.into()), B(vec![16])]);
        raw.values.extend([B(vec![10]), F(0.), B(vec![10]), F(1.)]);
        raw
    }

    #[test]
    fn discontinuity_lists_may_hold_a_subset_of_interior_knots() {
        // AutoCAD 22300 files list only some interior knots as third-derivative breaks.
        for breaks in [&[0.25, 0.5, 0.75][..], &[0.5], &[]] {
            let Some(Entity::BSplineCurve(c)) = decode(&cubic_curve(breaks), 22300).unwrap() else {
                panic!("{breaks:?}")
            };
            assert_eq!(c.poles.len(), 7);
        }
        for breaks in [&[0.6][..], &[0.75, 0.5], &[0.0], &[0.25, 0.25]] {
            assert!(decode(&cubic_curve(breaks), 22300).is_err(), "{breaks:?}");
        }
    }

    #[test]
    fn reversed_surface_sense_and_singular_v_boundary_keep_the_chart() {
        // AutoCAD 22300 files save reversed spline surfaces and V-singular (collapsed) boundaries.
        let forward = surface();
        let mut changed = forward.clone();
        changed.values[1] = AcisValue::Bytes(vec![10]);
        changed.values[12] = AcisValue::Integer(1.into());
        let (Some(Entity::BSplineSurface(a)), Some(Entity::BSplineSurface(b))) = (
            decode(&forward, 22700).unwrap(),
            decode(&changed, 22300).unwrap(),
        ) else {
            panic!()
        };
        assert!(!a.reversed && b.reversed);
        assert_eq!(a.evaluate(0.5, 0.5).unwrap(), b.evaluate(0.5, 0.5).unwrap());
        let mut curve = cubic_curve(&[]);
        curve.values[1] = AcisValue::Bytes(vec![10]);
        assert!(
            decode(&curve, 22300).is_err(),
            "reversed curves are not qualified"
        );
    }
}
