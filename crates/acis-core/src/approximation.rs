//! Saved B-spline fits of procedural ASM subtypes (intersection curves, swept/skinned surfaces).
//!
//! ASM saves, inside each procedural subtype scope, a 3D B-spline fit with its fit tolerance. This module reads
//! only that fit. Supports, laws and the other procedural fields are not interpreted and stay in the raw record:
//! a fit is an approximation within `fit_tolerance`, never the exact procedural geometry.
use crate::{
    asm_nurbs::Reader, AcisValue, BSplineCurveEntity, BSplineSurfaceEntity, RawEntity, NULL_REF,
};

/// Procedural curve subtypes whose leading fit is read.
pub const CURVE_KINDS: [&str; 3] = ["exact_int_cur", "par_int_cur", "int_int_cur"];
/// Procedural surface subtypes whose own-level fit is read.
pub const SURFACE_KINDS: [&str; 3] = ["exact_spl_sur", "cyl_spl_sur", "skin_spl_sur"];

#[derive(Debug, Clone, PartialEq)]
pub struct CurveFit {
    pub kind: String,
    /// Saved closure: 0 open, 1 closed, 2 periodic. The poles are clamped in every case.
    pub closure: usize,
    /// The fit; `fit_tolerance` is the saved one.
    pub curve: BSplineCurveEntity,
}

impl CurveFit {
    /// Evaluates the fit. A periodic fit (closure 2) wraps the parameter into its knot domain: edges on periodic
    /// intersection curves may use parameters of another period.
    pub fn evaluate(&self, t: f64) -> Result<crate::Vec3, String> {
        let knots = &self.curve.knots;
        let (a, b) = (knots[0], knots[knots.len() - 1]);
        let t = if self.closure == 2 && (t < a || t > b) {
            a + (t - a).rem_euclid(b - a)
        } else {
            t
        };
        self.curve.evaluate(t)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceFit {
    pub kind: String,
    /// Saved U/V closures (0 open, 1 closed, 2 periodic); the poles are clamped in every case.
    pub closure: [usize; 2],
    pub surface: BSplineSurfaceEntity,
}

/// Saved UV curve of a pcurve, as a planar B-spline: pole (u, v, 0).
#[derive(Debug, Clone, PartialEq)]
pub struct UvFit {
    pub curve: BSplineCurveEntity,
    /// Saved by `exp_par_cur`; intersection curves save their UV curves without one.
    pub fit_tolerance: Option<f64>,
}

/// UV curve number `which` (1 or 2) of the subtype scope opened at `values[start]`: the single UV spline of an
/// `exp_par_cur`, or the UV curves saved after the 3D fit of an intersection curve (`None` when saved as nullbs).
pub fn uv_fit(raw: &RawEntity, start: usize, which: usize) -> Result<Option<UvFit>, String> {
    let (kind, end) = scope(&raw.values, start)?;
    let mut fit = if kind == "exp_par_cur" {
        if which != 1 {
            return Err("exp_par_cur has a single UV curve".into());
        }
        let pos = *own_level_splines(&raw.values, start, end)
            .first()
            .ok_or("exp_par_cur without a UV spline")?;
        read_uv(&raw.values, pos, true)?
    } else if CURVE_KINDS.contains(&kind.as_str()) {
        // Slot 0 is the 3D fit; slots 1 and 2 are the UV curves on the first and second support.
        let slots = own_level_words(&raw.values, start, end, &["nubs", "nurbs", "nullbs"]);
        if !(1..=2).contains(&which) || slots.len() != 3 {
            return Err("unqualified intersection UV curves".into());
        }
        let pos = slots[which];
        if matches!(&raw.values[pos], AcisValue::String(s) if s == "nullbs") {
            return Ok(None);
        }
        read_uv(&raw.values, pos, false)?
    } else {
        return Err(format!("{kind} has no UV curve"));
    };
    fit.curve.raw = raw.clone();
    Ok(Some(fit))
}

/// UV curve used by a pcurve record. `reversed` is the pcurve's own sense (sense flag, or a negative type).
#[derive(Debug, Clone, PartialEq)]
pub struct PcurveFit {
    pub uv: UvFit,
    pub reversed: bool,
    /// Added to every point of `uv` to get face-surface parameters.
    pub uv_shift: [f64; 2],
    pub definition: Option<crate::subtypes::SubtypeDefinition>,
}

/// Fit of a procedural subtype, as saved.
#[derive(Debug, Clone, PartialEq)]
pub enum Fit {
    Curve(CurveFit),
    Surface(SurfaceFit),
}

/// Fit used by one record. `reversed` is the record's own sense (a curve runs as C(-t); a surface keeps its UV
/// chart and flips its normal); `definition` is set when the record reaches the subtype through a `ref`.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordFit {
    pub fit: Fit,
    pub reversed: bool,
    pub definition: Option<crate::subtypes::SubtypeDefinition>,
}

/// Fit of the subtype scope opened at `values[start]`, curve or surface by kind.
pub fn fit(raw: &RawEntity, start: usize) -> Result<Fit, String> {
    match raw.values.get(start + 1) {
        Some(AcisValue::String(k)) if CURVE_KINDS.contains(&k.as_str()) => {
            Ok(Fit::Curve(curve_fit(raw, start)?))
        }
        Some(AcisValue::String(k)) if SURFACE_KINDS.contains(&k.as_str()) => {
            Ok(Fit::Surface(surface_fit(raw, start)?))
        }
        _ => Err("no procedural fit for this subtype".into()),
    }
}

fn tag(v: Option<&AcisValue>, t: u8) -> bool {
    matches!(v, Some(AcisValue::Bytes(b)) if b == &[t])
}

/// Kind and end (exclusive, after the closing tag) of the subtype scope opened at `start`.
fn scope(values: &[AcisValue], start: usize) -> Result<(String, usize), String> {
    if !tag(values.get(start), 15) {
        return Err("subtype scope expected".into());
    }
    let kind = match values.get(start + 1) {
        Some(AcisValue::String(s)) => s.clone(),
        _ => return Err("subtype scope without a name".into()),
    };
    let mut depth = 0usize;
    for (i, v) in values.iter().enumerate().skip(start) {
        if tag(Some(v), 15) {
            depth += 1;
        } else if tag(Some(v), 16) {
            depth -= 1;
            if depth == 0 {
                return Ok((kind, i + 1));
            }
        }
    }
    Err("unclosed subtype scope".into())
}

/// End (exclusive) of the subtype scope opened at `values[start]`.
pub(crate) fn scope_end(values: &[AcisValue], start: usize) -> Result<usize, String> {
    Ok(scope(values, start)?.1)
}

/// Positions of "nubs"/"nurbs" words at the scope's own level (nested scopes skipped).
fn own_level_splines(values: &[AcisValue], start: usize, end: usize) -> Vec<usize> {
    own_level_words(values, start, end, &["nubs", "nurbs"])
}

fn own_level_words(values: &[AcisValue], start: usize, end: usize, words: &[&str]) -> Vec<usize> {
    let mut depth = 0usize;
    let mut out = Vec::new();
    for (i, v) in values.iter().enumerate().take(end).skip(start) {
        if tag(Some(v), 15) {
            depth += 1;
        } else if tag(Some(v), 16) {
            depth = depth.saturating_sub(1);
        } else if depth == 1 && matches!(v, AcisValue::String(s) if words.contains(&s.as_str())) {
            out.push(i);
        }
    }
    out
}

fn rational(r: &mut Reader<'_>) -> Result<bool, String> {
    match r.take()? {
        AcisValue::String(s) if s == "nubs" => Ok(false),
        AcisValue::String(s) if s == "nurbs" => Ok(true),
        _ => Err("expected nubs/nurbs".into()),
    }
}

fn closure(r: &mut Reader<'_>) -> Result<usize, String> {
    let c = r.integer()?;
    if c > 2 {
        return Err("unknown spline closure".into());
    }
    Ok(c)
}

fn fit_tolerance(r: &mut Reader<'_>) -> Result<f64, String> {
    let t = r.number()?;
    if t < 0.0 {
        return Err("negative fit tolerance".into());
    }
    Ok(t)
}

fn read_curve(values: &[AcisValue], pos: usize) -> Result<(usize, BSplineCurveEntity), String> {
    let mut r = Reader { values, pos };
    let rational = rational(&mut r)?;
    let degree = r.integer()?;
    let closure = closure(&mut r)?;
    let n = r.integer()?;
    let (knots, multiplicities, count) = r.knots(degree, n)?;
    let (poles, weights) = r.poles(count, rational)?;
    let curve = BSplineCurveEntity {
        raw: RawEntity::new(0, "intcurve-curve"),
        pattern: NULL_REF,
        degree,
        knots,
        multiplicities,
        poles,
        weights,
        parameter_range: None,
        fit_tolerance: fit_tolerance(&mut r)?,
    };
    curve.validate()?;
    Ok((closure, curve))
}

/// UV spline, as a planar curve (u, v, 0); `with_tolerance` reads a trailing fit tolerance.
fn read_uv(values: &[AcisValue], pos: usize, with_tolerance: bool) -> Result<UvFit, String> {
    let mut r = Reader { values, pos };
    let rational = rational(&mut r)?;
    let degree = r.integer()?;
    closure(&mut r)?;
    let n = r.integer()?;
    let (knots, multiplicities, count) = r.knots(degree, n)?;
    if count > (values.len() - r.pos) / if rational { 3 } else { 2 } {
        return Err("truncated UV poles".into());
    }
    let mut poles = Vec::with_capacity(count);
    let mut weights = Vec::with_capacity(count);
    for _ in 0..count {
        poles.push((r.number()?, r.number()?, 0.0).into());
        weights.push(if rational { r.number()? } else { 1.0 });
    }
    let fit_tolerance = if with_tolerance {
        Some(fit_tolerance(&mut r)?)
    } else {
        None
    };
    let curve = BSplineCurveEntity {
        raw: RawEntity::new(0, "pcurve"),
        pattern: NULL_REF,
        degree,
        knots,
        multiplicities,
        poles,
        weights,
        parameter_range: None,
        fit_tolerance: fit_tolerance.unwrap_or(0.0),
    };
    curve.validate()?;
    Ok(UvFit {
        curve,
        fit_tolerance,
    })
}

fn read_surface(
    values: &[AcisValue],
    pos: usize,
) -> Result<([usize; 2], BSplineSurfaceEntity), String> {
    let mut r = Reader { values, pos };
    let rational = rational(&mut r)?;
    let u_degree = r.integer()?;
    let v_degree = r.integer()?;
    let closure = [closure(&mut r)?, closure(&mut r)?];
    // Singularities (0 none, 1 collapsed boundary) do not change the poles or their evaluation.
    for _ in 0..2 {
        if r.integer()? > 2 {
            return Err("unknown spline singularity".into());
        }
    }
    let un = r.integer()?;
    let vn = r.integer()?;
    let (u_knots, u_multiplicities, u_count) = r.knots(u_degree, un)?;
    let (v_knots, v_multiplicities, v_count) = r.knots(v_degree, vn)?;
    let count = u_count
        .checked_mul(v_count)
        .ok_or("surface pole overflow")?;
    let (poles, weights) = r.poles(count, rational)?;
    let surface = BSplineSurfaceEntity {
        raw: RawEntity::new(0, "spline-surface"),
        pattern: NULL_REF,
        u_degree,
        v_degree,
        u_knots,
        v_knots,
        u_multiplicities,
        v_multiplicities,
        u_count,
        v_count,
        poles,
        weights,
        reversed: false,
        u_range: None,
        v_range: None,
        fit_tolerance: fit_tolerance(&mut r)?,
    };
    surface.validate()?;
    Ok((closure, surface))
}

/// Saved fit of the procedural curve subtype whose scope opens at `values[start]`: the first spline at the
/// scope's own level. The returned curve carries `raw` (the record that holds the scope) for provenance.
pub fn curve_fit(raw: &RawEntity, start: usize) -> Result<CurveFit, String> {
    let (kind, end) = scope(&raw.values, start)?;
    if !CURVE_KINDS.contains(&kind.as_str()) {
        return Err(format!("{kind} is not a procedural curve subtype"));
    }
    let pos = *own_level_splines(&raw.values, start, end)
        .first()
        .ok_or("procedural curve without a saved fit")?;
    let (closure, mut curve) = read_curve(&raw.values, pos)?;
    curve.raw = raw.clone();
    Ok(CurveFit {
        kind,
        closure,
        curve,
    })
}

/// Saved fit of the procedural surface subtype whose scope opens at `values[start]`: the first spline at the
/// scope's own level that reads as a surface (curve definitions of skins/sweeps are nested deeper).
pub fn surface_fit(raw: &RawEntity, start: usize) -> Result<SurfaceFit, String> {
    let (kind, end) = scope(&raw.values, start)?;
    if !SURFACE_KINDS.contains(&kind.as_str()) {
        return Err(format!("{kind} is not a procedural surface subtype"));
    }
    let mut last = String::from("procedural surface without a saved fit");
    for pos in own_level_splines(&raw.values, start, end) {
        match read_surface(&raw.values, pos) {
            Ok((closure, mut surface)) => {
                surface.raw = raw.clone();
                return Ok(SurfaceFit {
                    kind,
                    closure,
                    surface,
                });
            }
            Err(e) => last = e,
        }
    }
    Err(last)
}

#[cfg(test)]
mod tests {
    use super::*;
    use AcisValue::{Bytes as B, Float as F, Integer as I, String as S};

    fn line_fit() -> Vec<AcisValue> {
        // nubs, degree 1, open, 2 knots, 2 poles, fit tolerance.
        vec![
            S("nubs".into()),
            I(1.into()),
            I(0.into()),
            I(2.into()),
            F(0.),
            I(1.into()),
            F(2.),
            I(1.into()),
            F(0.),
            F(0.),
            F(0.),
            F(2.),
            F(4.),
            F(6.),
            F(1e-6),
        ]
    }

    #[test]
    fn curve_fit_reads_the_leading_spline_and_skips_supports() {
        let mut raw = RawEntity::new(5, "intcurve-curve");
        raw.values = vec![
            AcisValue::Reference(NULL_REF),
            B(vec![11]),
            B(vec![15]),
            S("int_int_cur".into()),
        ];
        raw.values.extend([I(23100.into()), I(0.into())]);
        raw.values.extend(line_fit());
        // Support data after the fit, including a nested scope with its own spline: not interpreted.
        raw.values.extend([
            S("spline".into()),
            B(vec![11]),
            B(vec![15]),
            S("ref".into()),
            I(0.into()),
            B(vec![16]),
        ]);
        raw.values.extend([B(vec![16]), B(vec![11]), B(vec![11])]);
        let fit = curve_fit(&raw, 2).unwrap();
        assert_eq!((fit.kind.as_str(), fit.closure), ("int_int_cur", 0));
        assert_eq!(fit.curve.fit_tolerance, 1e-6);
        let mid = fit.curve.evaluate(1.0).unwrap();
        assert_eq!((mid.x, mid.y, mid.z), (1., 2., 3.));
        assert_eq!(fit.curve.raw, raw);
        assert!(surface_fit(&raw, 2).is_err());
        assert!(curve_fit(&raw, 1).is_err());
        let mut unclosed = raw.clone();
        unclosed.values.truncate(unclosed.values.len() - 3);
        assert!(curve_fit(&unclosed, 2).is_err());
    }

    #[test]
    fn surface_fit_skips_nested_curve_fits() {
        let mut raw = RawEntity::new(5, "spline-surface");
        raw.values = vec![
            AcisValue::Reference(NULL_REF),
            B(vec![11]),
            B(vec![15]),
            S("skin_spl_sur".into()),
        ];
        raw.values.push(I(23100.into()));
        // A nested section curve with its own fit, then an own-level curve-shaped spline that is not a surface.
        raw.values.extend([
            B(vec![15]),
            S("exact_int_cur".into()),
            I(23100.into()),
            I(0.into()),
        ]);
        raw.values.extend(line_fit());
        raw.values.push(B(vec![16]));
        raw.values.push(S("nubs".into()));
        raw.values.extend([S("null_law".into())]);
        // The surface: bilinear patch.
        raw.values.extend([
            S("nubs".into()),
            I(1.into()),
            I(1.into()),
            I(0.into()),
            I(0.into()),
        ]);
        raw.values
            .extend([I(0.into()), I(1.into()), I(2.into()), I(2.into())]);
        for _ in 0..2 {
            raw.values.extend([F(0.), I(1.into()), F(1.), I(1.into())]);
        }
        // U varies fastest, as in exact_spl_sur.
        for (x, y) in [(0., 0.), (1., 0.), (0., 1.), (1., 1.)] {
            raw.values.extend([F(x), F(y), F(0.)]);
        }
        raw.values.extend([F(1e-5), B(vec![16])]);
        let fit = surface_fit(&raw, 2).unwrap();
        assert_eq!(fit.kind, "skin_spl_sur");
        assert_eq!(
            (
                fit.surface.u_count,
                fit.surface.v_count,
                fit.surface.fit_tolerance
            ),
            (2, 2, 1e-5)
        );
        let p = fit.surface.evaluate(0.5, 0.25).unwrap();
        assert_eq!((p.x, p.y, p.z), (0.5, 0.25, 0.));
        assert!(curve_fit(&raw, 2).is_err());
    }

    fn uv_line() -> Vec<AcisValue> {
        // nubs, degree 1, open, 2 knots, UV poles (0.1, 0.2) -> (0.3, 0.4).
        vec![
            S("nubs".into()),
            I(1.into()),
            I(0.into()),
            I(2.into()),
            F(0.),
            I(1.into()),
            F(1.),
            I(1.into()),
            F(0.1),
            F(0.2),
            F(0.3),
            F(0.4),
        ]
    }

    #[test]
    fn exp_par_cur_reads_its_uv_spline_and_fit_tolerance() {
        let mut raw = RawEntity::new(1, "pcurve");
        raw.values = vec![
            AcisValue::Reference(NULL_REF),
            I(0.into()),
            B(vec![11]),
            B(vec![15]),
            S("exp_par_cur".into()),
        ];
        raw.values.extend(uv_line());
        raw.values.push(F(1e-9));
        raw.values.extend([
            S("spline".into()),
            B(vec![11]),
            B(vec![15]),
            S("ref".into()),
            I(0.into()),
            B(vec![16]),
        ]);
        raw.values.extend([
            B(vec![11]),
            B(vec![11]),
            B(vec![11]),
            B(vec![11]),
            B(vec![16]),
            F(0.),
            F(0.),
        ]);
        let fit = uv_fit(&raw, 3, 1).unwrap().unwrap();
        assert_eq!(fit.fit_tolerance, Some(1e-9));
        let p = fit.curve.evaluate(0.5).unwrap();
        assert!((p.x - 0.2).abs() < 1e-15 && (p.y - 0.3).abs() < 1e-15 && p.z == 0.);
        assert!(
            uv_fit(&raw, 3, 2).is_err(),
            "exp_par_cur has a single UV curve"
        );
    }

    #[test]
    fn intersection_curves_hold_their_uv_curves_after_the_3d_fit() {
        let mut raw = RawEntity::new(5, "intcurve-curve");
        raw.values = vec![
            AcisValue::Reference(NULL_REF),
            B(vec![11]),
            B(vec![15]),
            S("par_int_cur".into()),
        ];
        raw.values.extend([I(23100.into()), I(0.into())]);
        raw.values.extend(line_fit());
        raw.values
            .extend([S("plane".into()), S("null_surface".into())]);
        raw.values.extend(uv_line());
        raw.values.push(S("nullbs".into()));
        raw.values.extend([B(vec![16]), B(vec![11]), B(vec![11])]);
        let first = uv_fit(&raw, 2, 1).unwrap().unwrap();
        assert_eq!(first.fit_tolerance, None);
        assert_eq!(first.curve.poles.len(), 2);
        assert!(
            uv_fit(&raw, 2, 2).unwrap().is_none(),
            "nullbs: no second UV curve"
        );
        assert!(uv_fit(&raw, 2, 3).is_err());
    }

    #[test]
    fn periodic_fits_wrap_the_parameter_into_their_knot_domain() {
        let (_, curve) = read_curve(&line_fit(), 0).unwrap();
        let open = CurveFit {
            kind: "int_int_cur".into(),
            closure: 0,
            curve,
        };
        assert!(open.evaluate(2.5).is_err());
        let periodic = CurveFit { closure: 2, ..open };
        // Domain [0, 2]: 2.5 and -1.5 wrap to 0.5.
        let expected = periodic.evaluate(0.5).unwrap();
        assert_eq!(periodic.evaluate(2.5).unwrap(), expected);
        assert_eq!(periodic.evaluate(-1.5).unwrap(), expected);
    }
}
