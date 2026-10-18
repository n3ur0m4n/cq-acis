//! Observed open exp_par_cur charts on the same explicit spline.
//! This is a partial view, not a general pcurve or procedural surface decoder.
use crate::{
    subtypes::{SubtypeDefinition, SubtypeTable},
    AcisModel, AcisValue, RawEntity,
};

#[derive(Debug, Clone, PartialEq)]
pub struct LinearSurfacePcurve {
    pub raw: RawEntity,
    pub parameter_interval: [f64; 2],
    pub uv_endpoints: [[f64; 2]; 2],
    /// Saved bound, never used to enlarge the consuming model's tolerance.
    pub fit_tolerance: f64,
    pub support: SubtypeDefinition,
}

/// Bounded ASM 22700 non-rational UV spline. The poles and knots retain the
/// saved direction; `reversed` means C(-t). Multiplicities use the clamped
/// convention (saved endpoint multiplicities plus one), as for ASM 3D splines.
#[derive(Debug, Clone, PartialEq)]
pub struct SplineSurfacePcurve {
    pub raw: RawEntity,
    pub degree: usize,
    pub knots: Vec<f64>,
    pub multiplicities: Vec<usize>,
    pub poles: Vec<[f64; 2]>,
    pub reversed: bool,
    pub parameter_interval: [f64; 2],
    pub fit_tolerance: f64,
    /// Surface normal sense, not a change of UV chart.
    pub support_reversed: bool,
    pub support: SubtypeDefinition,
}

fn tag(values: &[AcisValue], i: usize, byte: u8) -> bool {
    matches!(values.get(i), Some(AcisValue::Bytes(b)) if b == &[byte])
}
fn word(values: &[AcisValue], i: usize, expected: &str) -> bool {
    matches!(values.get(i), Some(AcisValue::String(s)) if s == expected)
}
fn integer(values: &[AcisValue], i: usize, expected: i64) -> bool {
    matches!(values.get(i), Some(AcisValue::Integer(n)) if *n == expected.into())
}
fn number(values: &[AcisValue], i: usize) -> Result<f64, String> {
    match values.get(i) {
        Some(AcisValue::Float(f)) if f.is_finite() => Ok(*f),
        _ => Err("linear pcurve requires a finite saved double".into()),
    }
}

fn definition(table: &SubtypeTable, entity: usize, start: usize) -> Option<&SubtypeDefinition> {
    table
        .definitions
        .iter()
        .find(|d| d.entity_index == entity && d.value_start == start)
        .or_else(|| {
            table
                .references
                .iter()
                .find(|r| r.entity_index == entity && r.value_start == start)
                .and_then(|r| r.target)
                .and_then(|i| table.definitions.get(i))
        })
}

pub(crate) fn decode_spline(
    model: &AcisModel,
    table: &SubtypeTable,
    index: usize,
    surface: usize,
) -> Result<Option<SplineSurfacePcurve>, String> {
    if !crate::qualified_model(model) {
        return Ok(None);
    }
    let raw = model.entities().get(index).ok_or("missing pcurve")?.raw();
    let target = model
        .entities()
        .get(surface)
        .ok_or("missing pcurve surface")?
        .raw();
    let v = &raw.values;
    if raw.type_name != "pcurve"
        || target.type_name != "spline-surface"
        || !word(v, 4, "exp_par_cur")
        || !word(v, 5, "nubs")
        || !(integer(v, 6, 1) || integer(v, 6, 3))
        || !integer(v, 7, 0)
    {
        return Ok(None);
    }
    if !matches!(v.first(), Some(AcisValue::Reference(_)))
        || !integer(v, 1, 0)
        || !(tag(v, 2, 10) || tag(v, 2, 11))
        || !tag(v, 3, 15)
    {
        return Err("unqualified spline pcurve envelope".into());
    }
    let degree = if integer(v, 6, 1) { 1 } else { 3 };
    // Counts are bounded before allocation or arithmetic on untrusted input.
    let count = match v.get(8) {
        Some(AcisValue::Integer(n)) => usize::try_from(n).ok(),
        _ => None,
    }
    .filter(|n| (2..=4096).contains(n))
    .ok_or("invalid pcurve knot count")?;
    let mut knots = Vec::with_capacity(count);
    let mut multiplicities = Vec::with_capacity(count);
    for i in 0..count {
        knots.push(number(v, 9 + 2 * i)?);
        let mult = (1..=degree)
            .find(|n| integer(v, 10 + 2 * i, *n as i64))
            .ok_or("invalid pcurve knot multiplicity")?;
        if (i == 0 || i == count - 1) && mult != degree {
            return Err("unqualified pcurve endpoint convention".into());
        }
        multiplicities.push(mult + usize::from(i == 0 || i == count - 1));
    }
    if knots.windows(2).any(|k| k[0] >= k[1]) || !(knots[count - 1] - knots[0]).is_finite() {
        return Err("invalid pcurve knot interval".into());
    }
    let pole_count = multiplicities.iter().sum::<usize>() - degree - 1;
    let mut pos = 9 + 2 * count;
    if pole_count > v.len().saturating_sub(pos) / 2 {
        return Err("truncated pcurve poles".into());
    }
    let mut poles = Vec::with_capacity(pole_count);
    for _ in 0..pole_count {
        poles.push([number(v, pos)?, number(v, pos + 1)?]);
        pos += 2;
    }
    let fit_tolerance = number(v, pos)?;
    if fit_tolerance < 0. || poles.iter().all(|p| p == &poles[0]) {
        return Err("invalid pcurve poles or tolerance".into());
    }
    if !word(v, pos + 1, "spline")
        || !(tag(v, pos + 2, 10) || tag(v, pos + 2, 11))
        || !tag(v, pos + 3, 15)
    {
        return Err("unqualified pcurve support envelope".into());
    }
    let support_reversed = tag(v, pos + 2, 10);
    let support = definition(table, index, pos + 3).ok_or("unresolved pcurve support")?;
    let face_support = definition(table, surface, 2).ok_or("unresolved face spline support")?;
    if support.kind != "exact_spl_sur" || support != face_support {
        return Err("pcurve references a different spline support".into());
    }
    let end = if word(v, pos + 4, "ref") && tag(v, pos + 6, 16) {
        pos + 7
    } else if word(v, pos + 4, "exact_spl_sur") {
        support.value_end
    } else {
        return Err("unsupported pcurve support profile".into());
    };
    if v.len() != end + 7
        || !(0..4).all(|i| tag(v, end + i, 11))
        || !tag(v, end + 4, 16)
        || number(v, end + 5)? != 0.
        || number(v, end + 6)? != 0.
    {
        return Err("unqualified pcurve support ranges or chart shift".into());
    }
    let reversed = tag(v, 2, 10);
    let parameter_interval = if reversed {
        [-knots[count - 1], -knots[0]]
    } else {
        [knots[0], knots[count - 1]]
    };
    Ok(Some(SplineSurfacePcurve {
        raw: raw.clone(),
        degree,
        knots,
        multiplicities,
        poles,
        reversed,
        parameter_interval,
        fit_tolerance,
        support_reversed,
        support: support.clone(),
    }))
}

pub(crate) fn decode(
    model: &AcisModel,
    table: &SubtypeTable,
    index: usize,
    surface: usize,
) -> Result<Option<LinearSurfacePcurve>, String> {
    if !crate::qualified_model(model) {
        return Ok(None);
    }
    let raw = model.entities().get(index).ok_or("missing pcurve")?.raw();
    let target = model
        .entities()
        .get(surface)
        .ok_or("missing pcurve surface")?
        .raw();
    let v = &raw.values;
    if raw.type_name != "pcurve"
        || target.type_name != "spline-surface"
        || !tag(v, 2, 11)
        || !word(v, 4, "exp_par_cur")
        || !word(v, 5, "nubs")
        || !integer(v, 6, 1)
        || !integer(v, 7, 0)
        || !integer(v, 8, 2)
        || !word(v, 18, "spline")
        || !tag(v, 19, 11)
    {
        return Ok(None);
    }
    if !matches!(v.first(), Some(AcisValue::Reference(_)))
        || !integer(v, 1, 0)
        || !tag(v, 3, 15)
        || !integer(v, 10, 1)
        || !integer(v, 12, 1)
        || !tag(v, 20, 15)
    {
        return Err("unqualified linear pcurve envelope".into());
    }
    let support = definition(table, index, 20).ok_or("unresolved pcurve support")?;
    let face_support = definition(table, surface, 2).ok_or("unresolved face spline support")?;
    if support.kind != "exact_spl_sur" || support != face_support {
        return Err("pcurve references a different spline support".into());
    }
    let end = if word(v, 21, "ref") {
        if !tag(v, 23, 16) {
            return Err("invalid pcurve support reference".into());
        }
        24
    } else if word(v, 21, "exact_spl_sur") {
        support.value_end
    } else {
        return Err("unsupported pcurve support profile".into());
    };
    // Only an unchanged chart, unbounded embedded support and zero UV shift.
    if v.len() != end + 7
        || !(0..4).all(|i| tag(v, end + i, 11))
        || !tag(v, end + 4, 16)
        || number(v, end + 5)? != 0.
        || number(v, end + 6)? != 0.
    {
        return Err("unqualified pcurve support ranges or chart shift".into());
    }
    let parameter_interval = [number(v, 9)?, number(v, 11)?];
    let uv_endpoints = [
        [number(v, 13)?, number(v, 14)?],
        [number(v, 15)?, number(v, 16)?],
    ];
    let fit_tolerance = number(v, 17)?;
    if parameter_interval[0] >= parameter_interval[1]
        || !(parameter_interval[1] - parameter_interval[0]).is_finite()
        || uv_endpoints[0] == uv_endpoints[1]
        || fit_tolerance < 0.
    {
        return Err("invalid linear pcurve interval, poles or tolerance".into());
    }
    Ok(Some(LinearSurfacePcurve {
        raw: raw.clone(),
        parameter_interval,
        uv_endpoints,
        fit_tolerance,
        support: support.clone(),
    }))
}
