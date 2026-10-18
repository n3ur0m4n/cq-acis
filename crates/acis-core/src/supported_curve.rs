//! Bounded full-form ASM 22700 support-associated spline views.
//! Stored 3D fits are not exact intersections. Consumers must check both the
//! saved support/UV representation and the shared edge before using a view.
use crate::{
    asm_nurbs::Reader,
    subtypes::{SubtypeDefinition, SubtypeTable},
    *,
};

#[derive(Debug, Clone, PartialEq)]
pub struct UvSpline {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub multiplicities: Vec<usize>,
    pub poles: Vec<[f64; 2]>,
    pub weights: Vec<f64>,
}
#[derive(Debug, Clone, PartialEq)]
pub struct SupportedCurve {
    pub curve: BSplineCurveEntity,
    pub kind: String,
    pub support: Entity,
    pub secondary_support: Option<Entity>,
    pub support_definition: Option<SubtypeDefinition>,
    pub pcurve: UvSpline,
    pub value_start: usize,
    pub value_end: usize,
    /// Saved internal interval and numeric lists; no healing meaning inferred.
    pub support_range: Option<ParameterRange>,
    pub saved_lists: [Vec<f64>; 3],
}
fn vector(r: &mut Reader<'_>, tag: u8) -> Result<Vec3, String> {
    match r.take()? {
        AcisValue::Bytes(b) if b.len() == 25 && b[0] == tag => {
            let xyz: Vec<_> = b[1..]
                .chunks_exact(8)
                .map(|c| f64::from_le_bytes(c.try_into().unwrap()))
                .collect();
            if xyz.iter().any(|n| !n.is_finite()) {
                return Err("nonfinite support vector".into());
            }
            Ok((xyz[0], xyz[1], xyz[2]).into())
        }
        _ => Err("unqualified support vector tag".into()),
    }
}
fn surface(
    r: &mut Reader<'_>,
    raw: &RawEntity,
    table: Option<&SubtypeTable>,
) -> Result<(Entity, Option<SubtypeDefinition>), String> {
    let kind = match r.take()? {
        AcisValue::String(s) => s.clone(),
        _ => return Err("missing curve support".into()),
    };
    let empty = || {
        Some(ParameterRange {
            lower: None,
            upper: None,
        })
    };
    let geometry = match kind.as_str() {
        "plane" => Entity::PlaneSurface(PlaneSurfaceEntity {
            raw: raw.clone(),
            pattern: NULL_REF,
            origin: vector(r, 19)?,
            normal: vector(r, 20)?,
            u_direction: vector(r, 20)?,
            reverse_v: false,
            u_range: empty(),
            v_range: empty(),
        }),
        "sphere" => {
            let center = vector(r, 19)?;
            let radius = r.number()?;
            let u_direction = vector(r, 20)?;
            let pole = vector(r, 20)?;
            Entity::SphereSurface(SphereSurfaceEntity {
                raw: raw.clone(),
                pattern: NULL_REF,
                center,
                radius,
                pole,
                u_direction,
                reversed: false,
                u_range: empty(),
                v_range: empty(),
            })
        }
        "torus" => Entity::TorusSurface(TorusSurfaceEntity {
            raw: raw.clone(),
            pattern: NULL_REF,
            center: vector(r, 19)?,
            axis: vector(r, 20)?,
            major_radius: r.number()?,
            minor_radius: r.number()?,
            u_direction: vector(r, 20)?,
            reversed: false,
            u_range: empty(),
            v_range: empty(),
        }),
        "spline" => {
            r.byte(11)?;
            let start = r.pos;
            r.byte(15)?;
            r.word("ref")?;
            let index = r.integer()?;
            r.byte(16)?;
            let table = table.ok_or("spline support needs its source subtype table")?;
            let reference = table
                .references
                .iter()
                .find(|s| s.entity_index == raw.index && s.value_start == start)
                .ok_or("missing curve support reference")?;
            if reference.target != Some(index) {
                return Err("unresolved curve support reference".into());
            }
            let definition = table
                .definitions
                .get(index)
                .filter(|d| d.kind == "exact_spl_sur")
                .ok_or("unsupported curve support subtype")?
                .clone();
            // Expansion is performed by decode_with_model, which owns the source.
            for _ in 0..4 {
                r.byte(11)?;
            }
            return Ok((Entity::Raw(raw.clone()), Some(definition)));
        }
        _ => return Err("unqualified associated surface".into()),
    };
    r.byte(11)?; // Forward surface / non-reversed plane V.
    for _ in 0..4 {
        r.byte(11)?;
    }
    Ok((geometry, None))
}
fn uv(r: &mut Reader<'_>) -> Result<UvSpline, String> {
    let rational = match r.take()? {
        AcisValue::String(s) if s == "nubs" => false,
        AcisValue::String(s) if s == "nurbs" => true,
        _ => return Err("unqualified support UV representation".into()),
    };
    let degree = r.integer()?;
    if !matches!((degree, rational), (3, false) | (2, true)) {
        return Err("unqualified support UV degree".into());
    }
    r.int(0)?;
    let count = r.integer()?;
    let (knots, multiplicities, poles_count) = r.knots(degree, count)?;
    let width = if rational { 3 } else { 2 };
    if poles_count > r.values.len().saturating_sub(r.pos) / width {
        return Err("truncated support UV poles".into());
    }
    let mut poles = Vec::with_capacity(poles_count);
    let mut weights = Vec::with_capacity(poles_count);
    for _ in 0..poles_count {
        poles.push([r.number()?, r.number()?]);
        let w = if rational { r.number()? } else { 1. };
        if w <= 0. {
            return Err("nonpositive support UV weight".into());
        }
        weights.push(w);
    }
    Ok(UvSpline {
        degree,
        knots,
        multiplicities,
        poles,
        weights,
    })
}
fn list(r: &mut Reader<'_>, lower: f64, upper: f64) -> Result<Vec<f64>, String> {
    let n = r.integer()?;
    if n > 4096 || n > r.values.len().saturating_sub(r.pos) {
        return Err("invalid support list count".into());
    }
    let values = (0..n).map(|_| r.number()).collect::<Result<Vec<_>, _>>()?;
    if values.iter().any(|v| *v < lower || *v > upper) || values.windows(2).any(|v| v[0] >= v[1]) {
        return Err("invalid support list parameters".into());
    }
    Ok(values)
}
/// Decode just the observed full representation. Never drop unknown supports,
/// absent UVs, curve senses, closure flags or trailing fields.
pub(crate) fn decode(
    raw: &RawEntity,
    start: usize,
    table: Option<&SubtypeTable>,
) -> Result<SupportedCurve, String> {
    let mut r = Reader {
        values: &raw.values,
        pos: start,
    };
    r.byte(11)?;
    r.byte(15)?;
    let kind = match r.take()? {
        AcisValue::String(s) if s == "par_int_cur" || s == "int_int_cur" => s.clone(),
        _ => return Err("unqualified supported curve kind".into()),
    };
    if (start == 14) != (kind == "par_int_cur") {
        return Err("unqualified supported curve owner".into());
    }
    r.embedded_version()?;
    r.int(0)?;
    r.word("nubs")?;
    r.int(3)?;
    r.int(0)?;
    let n = r.integer()?;
    let (knots, multiplicities, count) = r.knots(3, n)?;
    let (poles, weights) = r.poles(count, false)?;
    let fit_tolerance = r.number()?;
    if fit_tolerance < 0. {
        return Err("negative supported curve fit tolerance".into());
    }
    let (support, support_definition) = surface(&mut r, raw, table)?;
    let secondary_support = if kind == "par_int_cur" {
        r.word("null_surface")?;
        None
    } else {
        let (s, d) = surface(&mut r, raw, table)?;
        if !matches!(s, Entity::PlaneSurface(_)) || d.is_some() {
            return Err("unqualified second intersection support".into());
        }
        Some(s)
    };
    if kind == "int_int_cur" && support_definition.is_none() {
        return Err("unqualified first intersection support".into());
    }
    let pcurve = uv(&mut r)?;
    if kind == "int_int_cur" && pcurve.degree != 3 {
        return Err("unqualified intersection UV profile".into());
    }
    if pcurve.knots.first() != knots.first() || pcurve.knots.last() != knots.last() {
        return Err("support UV and 3D intervals differ".into());
    }
    r.word("nullbs")?;
    let support_range = r.range()?;
    if let Some(range) = &support_range {
        if [range.lower, range.upper]
            .into_iter()
            .flatten()
            .any(|t| t < knots[0] || t > knots[n - 1])
            || matches!((range.lower, range.upper), (Some(a), Some(b)) if a > b)
        {
            return Err("invalid associated support interval".into());
        }
    }
    let saved_lists = [
        list(&mut r, knots[0], knots[n - 1])?,
        list(&mut r, knots[0], knots[n - 1])?,
        list(&mut r, knots[0], knots[n - 1])?,
    ];
    if !saved_lists[0].is_empty() {
        return Err("unqualified support list profile".into());
    }
    r.int(0)?;
    if kind == "par_int_cur" {
        if saved_lists[1] != pcurve.knots[1..pcurve.knots.len() - 1]
            || !saved_lists[2].is_empty()
            || support_range
                .as_ref()
                .is_some_and(|r| r.lower.is_some() || r.upper.is_some())
        {
            return Err("unqualified inline support trailer".into());
        }
        r.byte(10)?;
        r.byte(11)?;
    }
    r.byte(16)?;
    let parameter_range = r.range()?;
    if kind == "par_int_cur"
        && parameter_range
            .as_ref()
            .is_some_and(|r| r.lower.is_some() || r.upper.is_some())
    {
        return Err("unqualified inline outer interval".into());
    }
    let curve = BSplineCurveEntity {
        raw: raw.clone(),
        pattern: NULL_REF,
        degree: 3,
        knots,
        multiplicities,
        poles,
        weights,
        parameter_range,
        fit_tolerance,
    };
    curve.validate()?;
    Ok(SupportedCurve {
        curve,
        kind,
        support,
        secondary_support,
        support_definition,
        pcurve,
        value_start: start,
        value_end: r.pos,
        support_range,
        saved_lists,
    })
}

pub(crate) fn decode_with_model(
    model: &AcisModel,
    table: &SubtypeTable,
    index: usize,
) -> Result<Option<SupportedCurve>, String> {
    if !crate::qualified_model(model) {
        return Ok(None);
    }
    let raw = model
        .entities()
        .get(index)
        .ok_or("missing supported curve")?
        .raw();
    if raw.type_name != "intcurve-curve"
        || !matches!(raw.values.get(3),Some(AcisValue::String(s)) if s=="int_int_cur")
    {
        return Ok(None);
    }
    if !matches!(raw.values.first(), Some(AcisValue::Reference(_))) {
        return Err("invalid supported curve pattern reference".into());
    }
    let mut view = decode(raw, 1, Some(table))?;
    if let Some(AcisValue::Reference(pattern)) = raw.values.first() {
        view.curve.pattern = *pattern;
    }
    if view.value_end != raw.values.len() {
        return Err("unconsumed supported curve fields".into());
    }
    let d = view
        .support_definition
        .as_ref()
        .ok_or("missing intersection support")?;
    let owner = model
        .entities()
        .get(d.entity_index)
        .ok_or("missing support owner")?
        .raw();
    let scope = owner
        .values
        .get(d.value_start..d.value_end)
        .ok_or("invalid support extent")?;
    let mut expanded = raw.clone();
    expanded.type_name = "spline-surface".into();
    expanded.values = vec![AcisValue::Reference(NULL_REF), AcisValue::Bytes(vec![11])];
    expanded.values.extend_from_slice(scope);
    expanded
        .values
        .extend((0..4).map(|_| AcisValue::Bytes(vec![11])));
    let mut geometry =
        crate::asm_nurbs::decode(&expanded, 22700)?.ok_or("support surface not decoded")?;
    if let Entity::BSplineSurface(s) = &mut geometry {
        s.raw = raw.clone();
    } else {
        return Err("invalid support surface".into());
    }
    view.support = geometry;
    Ok(Some(view))
}
