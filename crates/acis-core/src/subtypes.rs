//! File-local subtype indexing for a bounded ASM 22700 vocabulary.
//! Definitions receive indices on opening, including nested definitions.
//! Unknown scopes stop reliable numbering; references never consume an index.
use crate::{AcisModel, AcisValue, Entity};
use std::{collections::BTreeMap, sync::Arc};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SubtypeDefinition {
    pub index: usize,
    pub kind: String,
    pub entity_index: usize,
    /// Half-open indices into RawEntity.values, including scope delimiters.
    pub value_start: usize,
    pub value_end: usize,
    pub parent: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SubtypeReference {
    pub entity_index: usize,
    pub value_start: usize,
    pub target: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SubtypeTable {
    pub definitions: Vec<SubtypeDefinition>,
    pub references: Vec<SubtypeReference>,
    pub diagnostics: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedSubtype {
    /// A view with the referencing entity's original raw bytes and span.
    pub geometry: Entity,
    pub definition: SubtypeDefinition,
}

fn tag(v: &AcisValue, t: u8) -> bool {
    matches!(v, AcisValue::Bytes(b) if b == &[t])
}
fn word(v: Option<&AcisValue>) -> Option<&str> {
    match v {
        Some(AcisValue::String(s)) => Some(s),
        _ => None,
    }
}
fn vocabulary(s: &str) -> bool {
    matches!(
        s,
        "exact_spl_sur"
            | "exact_int_cur"
            | "exp_par_cur"
            | "par_int_cur"
            | "int_int_cur"
            | "cyl_spl_sur"
            | "skin_spl_sur"
    )
}

fn root_scope(raw: &crate::RawEntity, start: usize, kind: Option<&str>) -> bool {
    let boolean = |i| raw.values.get(i).is_some_and(|v| tag(v, 10) || tag(v, 11));
    let integer =
        |i, n: i64| matches!(raw.values.get(i), Some(AcisValue::Integer(v)) if *v == n.into());
    let reference = |i| matches!(raw.values.get(i), Some(AcisValue::Reference(_)));
    if !reference(0) {
        return false;
    }
    match raw.type_name.as_str() {
        "spline-surface" => {
            start == 2
                && boolean(1)
                && matches!(
                    kind,
                    Some("exact_spl_sur" | "cyl_spl_sur" | "skin_spl_sur" | "ref")
                )
        }
        "intcurve-curve" => {
            start == 2
                && boolean(1)
                && matches!(
                    kind,
                    Some("exact_int_cur" | "par_int_cur" | "int_int_cur" | "ref")
                )
        }
        "pcurve" => {
            start == 3 && integer(1, 0) && boolean(2) && matches!(kind, Some("exp_par_cur" | "ref"))
        }
        "tcoedge-coedge" => {
            start == 15
                && reference(11)
                && integer(12, 1)
                && word(raw.values.get(13)) == Some("intcurve")
                && boolean(14)
                && matches!(
                    kind,
                    Some("exact_int_cur" | "par_int_cur" | "int_int_cur" | "ref")
                )
        }
        _ => false,
    }
}

/// Index only an observed subtype vocabulary, without interpreting procedural
/// geometry. Unknown braces may have different registration rules, so every
/// reference after the first unknown/malformed scope is left unresolved.
pub fn index(model: &AcisModel) -> SubtypeTable {
    let mut table = SubtypeTable::default();
    if !crate::qualified_model(model) {
        table
            .diagnostics
            .push("subtype table requires a qualified ASM save version".into());
        return table;
    }
    let mut reliable = true;
    for entity in model.entities() {
        let raw = entity.raw();
        let mut stack: Vec<Option<usize>> = Vec::new();
        let mut root_seen = false;
        for (i, value) in raw.values.iter().enumerate() {
            if tag(value, 15) {
                let kind = word(raw.values.get(i + 1));
                if stack.is_empty() {
                    if reliable && (root_seen || !root_scope(raw, i, kind)) {
                        reliable = false;
                        table.diagnostics.push(format!(
                            "entity {} value {i}: unqualified subtype record envelope",
                            raw.index
                        ));
                    }
                    root_seen = true;
                }
                if kind == Some("ref") {
                    if !raw.values.get(i + 3).is_some_and(|v| tag(v, 16)) {
                        reliable = false;
                    }
                    let target = if reliable && raw.values.get(i + 3).is_some_and(|v| tag(v, 16)) {
                        match raw.values.get(i + 2) {
                            Some(AcisValue::Integer(n)) => {
                                n.to_string().parse::<usize>().ok().filter(|n| {
                                    table.definitions.get(*n).is_some_and(|d| d.value_end != 0)
                                })
                            }
                            _ => None,
                        }
                    } else {
                        None
                    };
                    if target.is_none() {
                        table.diagnostics.push(format!(
                            "entity {} value {i}: unresolved subtype reference",
                            raw.index
                        ));
                    }
                    table.references.push(SubtypeReference {
                        entity_index: raw.index,
                        value_start: i,
                        target,
                    });
                    stack.push(None);
                } else if reliable && kind.is_some_and(vocabulary) {
                    let slot = table.definitions.len();
                    table.definitions.push(SubtypeDefinition {
                        index: slot,
                        kind: kind.unwrap().into(),
                        entity_index: raw.index,
                        value_start: i,
                        value_end: 0,
                        parent: stack.iter().rev().flatten().next().copied(),
                    });
                    stack.push(Some(slot));
                } else {
                    if reliable {
                        table.diagnostics.push(format!("entity {} value {i}: unknown scope; subsequent subtype numbering is unqualified", raw.index));
                    }
                    reliable = false;
                    stack.push(None);
                }
            } else if tag(value, 16) {
                match stack.pop() {
                    Some(Some(slot)) => table.definitions[slot].value_end = i + 1,
                    Some(None) => (),
                    None => {
                        reliable = false;
                        table.diagnostics.push(format!(
                            "entity {} value {i}: unmatched subtype close",
                            raw.index
                        ));
                    }
                }
            }
        }
        if !stack.is_empty() {
            reliable = false;
            table
                .diagnostics
                .push(format!("entity {}: unclosed subtype", raw.index));
        }
    }
    table
}

/// An index bound to its immutable source model; indices cannot cross files.
/// Build once when resolving multiple aliases.
pub struct SubtypeResolver {
    model: Arc<AcisModel>,
    table: SubtypeTable,
    roots: BTreeMap<usize, usize>,
}
impl SubtypeResolver {
    pub fn new(model: Arc<AcisModel>) -> Self {
        let table = index(&model);
        let roots = table
            .references
            .iter()
            .enumerate()
            .filter(|(_, r)| r.value_start == 2)
            .map(|(i, r)| (r.entity_index, i))
            .collect();
        Self {
            model,
            table,
            roots,
        }
    }
    pub fn table(&self) -> &SubtypeTable {
        &self.table
    }
    pub fn resolve(&self, entity_index: usize) -> Result<Option<ResolvedSubtype>, String> {
        resolve(
            &self.model,
            &self.table,
            self.roots.get(&entity_index).copied(),
            entity_index,
        )
    }
    pub fn linear_surface_pcurve(
        &self,
        pcurve: usize,
        surface: usize,
    ) -> Result<Option<crate::pcurve::LinearSurfacePcurve>, String> {
        crate::pcurve::decode(&self.model, &self.table, pcurve, surface)
    }
    pub fn spline_surface_pcurve(
        &self,
        pcurve: usize,
        surface: usize,
    ) -> Result<Option<crate::pcurve::SplineSurfacePcurve>, String> {
        crate::pcurve::decode_spline(&self.model, &self.table, pcurve, surface)
    }
    /// Saved fit of a procedural spline-surface, intcurve or tolerant-coedge inline curve, following a `ref`
    /// to its definition. `None` for other records. See [`crate::approximation`].
    pub fn fit(
        &self,
        entity_index: usize,
    ) -> Result<Option<crate::approximation::RecordFit>, String> {
        use crate::approximation::Fit;
        let raw = self
            .model
            .entities()
            .get(entity_index)
            .ok_or("missing fit owner")?
            .raw();
        let (start, surface) = match raw.type_name.as_str() {
            "spline-surface" => (2, true),
            "intcurve-curve" => (2, false),
            "tcoedge-coedge" if word(raw.values.get(13)) == Some("intcurve") => (15, false),
            _ => return Ok(None),
        };
        let reversed = match raw.values.get(start - 1) {
            Some(v) if tag(v, 10) => true,
            Some(v) if tag(v, 11) => false,
            _ => return Err("missing subtype sense".into()),
        };
        let (owner, scope_start, definition) = if word(raw.values.get(start + 1)) == Some("ref") {
            let definition = self
                .table
                .references
                .iter()
                .find(|r| r.entity_index == entity_index && r.value_start == start)
                .and_then(|r| r.target)
                .and_then(|i| self.table.definitions.get(i))
                .ok_or("unresolved subtype index")?;
            let owner = self
                .model
                .entities()
                .get(definition.entity_index)
                .ok_or("missing subtype owner")?
                .raw();
            (owner, definition.value_start, Some(definition.clone()))
        } else {
            (raw, start, None)
        };
        let fit = crate::approximation::fit(owner, scope_start)?;
        if matches!(fit, Fit::Surface(_)) != surface {
            return Err("subtype kind does not match its record".into());
        }
        Ok(Some(crate::approximation::RecordFit {
            fit,
            reversed,
            definition,
        }))
    }
    /// Saved UV curve of a pcurve record: its own `exp_par_cur` (or a `ref` to one), or UV curve 1/2 of the
    /// intersection curve it names (type +-1/+-2). `None` for other records. The UV chart is the one of the face
    /// surface, after adding the saved UV shift. See [`crate::approximation`].
    pub fn pcurve_fit(
        &self,
        entity_index: usize,
    ) -> Result<Option<crate::approximation::PcurveFit>, String> {
        let raw = self
            .model
            .entities()
            .get(entity_index)
            .ok_or("missing pcurve")?
            .raw();
        if raw.type_name != "pcurve" {
            return Ok(None);
        }
        let v = &raw.values;
        let integer = |i: usize| match v.get(i) {
            Some(AcisValue::Integer(n)) => n.to_string().parse::<i64>().ok(),
            _ => None,
        };
        let number = |i: usize| match v.get(i) {
            Some(AcisValue::Float(x)) if x.is_finite() => Some(*x),
            _ => None,
        };
        let kind = integer(1).ok_or("pcurve without a type")?;
        let (owner, start, which, reversed, shift_at) = if kind == 0 {
            let reversed = match v.get(2) {
                Some(x) if tag(x, 10) => true,
                Some(x) if tag(x, 11) => false,
                _ => return Err("missing pcurve sense".into()),
            };
            let end = crate::approximation::scope_end(v, 3)?;
            (raw, 3, 1, reversed, end)
        } else if (1..=2).contains(&kind.abs()) {
            let curve = match v.get(2) {
                Some(AcisValue::Reference(r)) => crate::resolve_index(*r, self.model.len())
                    .map_err(|e| e.to_string())?
                    .ok_or("null pcurve intcurve")?,
                _ => return Err("pcurve without its intcurve".into()),
            };
            let owner = self.model.entities()[curve].raw();
            if owner.type_name != "intcurve-curve" {
                return Err("pcurve names a non-intcurve record".into());
            }
            (owner, 2, kind.unsigned_abs() as usize, kind < 0, 3)
        } else {
            return Err("unqualified pcurve type".into());
        };
        // Saved UV shift: a whole period on periodic analytic charts in AutoCAD files (e.g. 0, 2pi on tori).
        let uv_shift = match (number(shift_at), number(shift_at + 1)) {
            (Some(du), Some(dv)) if v.len() == shift_at + 2 => [du, dv],
            _ => return Err("unqualified pcurve UV shift or trailer".into()),
        };
        // Follow a ref to the subtype definition.
        let (owner, start, definition) = if word(owner.values.get(start + 1)) == Some("ref") {
            let definition = self
                .table
                .references
                .iter()
                .find(|r| r.entity_index == owner.index && r.value_start == start)
                .and_then(|r| r.target)
                .and_then(|i| self.table.definitions.get(i))
                .ok_or("unresolved subtype index")?;
            let defining = self.model.entities()[definition.entity_index].raw();
            (defining, definition.value_start, Some(definition.clone()))
        } else {
            (owner, start, None)
        };
        let uv = crate::approximation::uv_fit(owner, start, which)?
            .ok_or("pcurve names an absent intcurve UV curve")?;
        Ok(Some(crate::approximation::PcurveFit {
            uv,
            reversed,
            uv_shift,
            definition,
        }))
    }
    pub fn supported_curve(
        &self,
        index: usize,
    ) -> Result<Option<crate::supported_curve::SupportedCurve>, String> {
        crate::supported_curve::decode_with_model(&self.model, &self.table, index)
    }
}

/// Resolve a top-level geometry alias through the indexed definition. Nested
/// definitions are eligible, but only the existing fully validated explicit
/// spline grammar can produce geometry. Saved sense/ranges belong to the user
/// of the subtype, not to the record containing its definition.
fn resolve(
    model: &AcisModel,
    table: &SubtypeTable,
    reference_index: Option<usize>,
    entity_index: usize,
) -> Result<Option<ResolvedSubtype>, String> {
    let raw = model
        .entities()
        .get(entity_index)
        .ok_or("missing subtype reference owner")?
        .raw();
    let expected = match raw.type_name.as_str() {
        "spline-surface" => "exact_spl_sur",
        "intcurve-curve" => "exact_int_cur",
        _ => return Ok(None),
    };
    if word(raw.values.get(3)) != Some("ref") {
        return Ok(None);
    }
    let reference = reference_index
        .and_then(|i| table.references.get(i))
        .ok_or("missing qualified subtype reference")?;
    let definition = reference
        .target
        .and_then(|i| table.definitions.get(i))
        .ok_or("unresolved subtype index")?;
    if definition.kind != expected {
        return Err("subtype reference has an unsupported geometry kind".into());
    }
    let owner = model
        .entities()
        .get(definition.entity_index)
        .ok_or("missing subtype owner")?
        .raw();
    let scope = owner
        .values
        .get(definition.value_start..definition.value_end)
        .ok_or("invalid subtype extent")?;
    if raw.values.len() < 6 || !tag(&raw.values[2], 15) || !tag(&raw.values[5], 16) {
        return Err("invalid geometry alias envelope".into());
    }
    let mut expanded = raw.clone();
    expanded.values = raw.values[..2]
        .iter()
        .chain(scope)
        .chain(&raw.values[6..])
        .cloned()
        .collect();
    let mut geometry =
        crate::asm_nurbs::decode(&expanded, 22700)?.ok_or("subtype geometry not decoded")?;
    // Never label expanded bytes as original source bytes.
    match &mut geometry {
        Entity::BSplineSurface(s) => s.raw = raw.clone(),
        Entity::BSplineCurve(c) => c.raw = raw.clone(),
        _ => return Err("unexpected resolved geometry".into()),
    }
    Ok(Some(ResolvedSubtype {
        geometry,
        definition: definition.clone(),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AcisMetadata, RawEntity};
    fn b(t: u8) -> AcisValue {
        AcisValue::Bytes(vec![t])
    }
    fn s(t: &str) -> AcisValue {
        AcisValue::String(t.into())
    }
    fn model(values: Vec<AcisValue>) -> AcisModel {
        let mut raw = RawEntity::new(0, "pcurve");
        raw.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            AcisValue::Integer(0.into()),
            b(11),
        ];
        raw.values.extend(values);
        AcisModel::new(
            AcisMetadata {
                save_version: Some(22700.into()),
                ..Default::default()
            },
            vec![Entity::Raw(raw)],
            vec![],
        )
        .unwrap()
    }
    #[test]
    fn nested_definitions_use_preorder_and_refs_do_not_allocate() {
        let m = model(vec![
            b(15),
            s("exp_par_cur"),
            b(15),
            s("exact_spl_sur"),
            b(16),
            b(15),
            s("ref"),
            AcisValue::Integer(1.into()),
            b(16),
            b(16),
        ]);
        let t = index(&m);
        assert_eq!(t.definitions.len(), 2);
        assert_eq!(t.definitions[1].parent, Some(0));
        assert_eq!(t.definitions[1].value_start, 5);
        assert_eq!(t.references[0].target, Some(1));
        assert!(t.diagnostics.is_empty());
    }
    #[test]
    fn unknown_scopes_forward_and_recursive_refs_are_not_guessed() {
        for values in [
            vec![b(15), s("ref"), AcisValue::Integer(0.into()), b(16)],
            vec![
                b(15),
                s("exact_spl_sur"),
                b(15),
                s("ref"),
                AcisValue::Integer(0.into()),
                b(16),
                b(16),
            ],
            vec![
                b(15),
                s("unknown"),
                b(16),
                b(15),
                s("exact_spl_sur"),
                b(16),
                b(15),
                s("ref"),
                AcisValue::Integer(0.into()),
                b(16),
            ],
        ] {
            let t = index(&model(values));
            assert!(t.references.iter().all(|r| r.target.is_none()));
            assert!(!t.diagnostics.is_empty());
        }
    }
    #[test]
    fn swept_and_skinned_surfaces_are_numbered_and_their_fits_resolve_through_refs() {
        use crate::approximation::Fit;
        let fit = |kind: &str| {
            let mut v = vec![b(15), s(kind), AcisValue::Integer(23100.into())];
            v.extend([
                s("nubs"),
                AcisValue::Integer(1.into()),
                AcisValue::Integer(1.into()),
            ]);
            v.extend((0..4).map(|_| AcisValue::Integer(0.into())));
            v.extend([AcisValue::Integer(2.into()), AcisValue::Integer(2.into())]);
            for _ in 0..2 {
                v.extend([
                    AcisValue::Float(0.),
                    AcisValue::Integer(1.into()),
                    AcisValue::Float(1.),
                    AcisValue::Integer(1.into()),
                ]);
            }
            for (x, y) in [(0., 0.), (1., 0.), (0., 1.), (1., 1.)] {
                v.extend([
                    AcisValue::Float(x),
                    AcisValue::Float(y),
                    AcisValue::Float(0.),
                ]);
            }
            v.extend([AcisValue::Float(0.), b(16)]);
            v
        };
        let surface = |index, scope: Vec<AcisValue>, sense| {
            let mut raw = RawEntity::new(index, "spline-surface");
            raw.values = vec![AcisValue::Reference(crate::NULL_REF), b(sense)];
            raw.values.extend(scope);
            raw.values.extend((0..4).map(|_| b(11)));
            Entity::Raw(raw)
        };
        let reference = vec![b(15), s("ref"), AcisValue::Integer(1.into()), b(16)];
        let model = AcisModel::new(
            AcisMetadata {
                save_version: Some(22300.into()),
                ..Default::default()
            },
            vec![
                surface(0, fit("cyl_spl_sur"), 11),
                surface(1, fit("skin_spl_sur"), 11),
                surface(2, reference, 10),
            ],
            vec![],
        )
        .unwrap();
        let resolver = SubtypeResolver::new(Arc::new(model));
        assert!(
            resolver.table().diagnostics.is_empty(),
            "{:?}",
            resolver.table().diagnostics
        );
        let r = resolver.fit(2).unwrap().unwrap();
        assert!(r.reversed);
        assert_eq!(r.definition.as_ref().map(|d| d.entity_index), Some(1));
        let Fit::Surface(f) = r.fit else { panic!() };
        assert_eq!(f.kind, "skin_spl_sur");
        assert!(!resolver.fit(0).unwrap().unwrap().reversed);
    }
    #[test]
    fn pcurves_reach_their_uv_curve_directly_through_refs_or_through_an_intcurve() {
        let int = |n: i64| AcisValue::Integer(n.into());
        let f = AcisValue::Float;
        let uv = |shift: f64| {
            vec![
                s("nubs"),
                int(1),
                int(0),
                int(2),
                f(0.),
                int(1),
                f(1.),
                int(1),
                f(shift),
                f(0.2),
                f(0.3),
                f(0.4),
            ]
        };
        // 0: pcurve with its own exp_par_cur.
        let mut own = RawEntity::new(0, "pcurve");
        own.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            int(0),
            b(10),
            b(15),
            s("exp_par_cur"),
        ];
        own.values.extend(uv(0.1));
        own.values
            .extend([f(0.), s("null_surface"), b(16), f(0.), f(0.)]);
        // 1: pcurve referring to definition 0.
        let mut alias = RawEntity::new(1, "pcurve");
        alias.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            int(0),
            b(11),
            b(15),
            s("ref"),
            int(0),
            b(16),
        ];
        alias.values.extend([f(0.), f(0.)]);
        // 2: intersection curve with a 3D fit and one UV curve; 3: pcurve on its first UV curve, reversed.
        let mut curve = RawEntity::new(2, "intcurve-curve");
        curve.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            b(11),
            b(15),
            s("par_int_cur"),
            int(23100),
            int(0),
        ];
        curve.values.extend([
            s("nubs"),
            int(1),
            int(0),
            int(2),
            f(0.),
            int(1),
            f(1.),
            int(1),
        ]);
        curve.values.extend([
            f(0.),
            f(0.),
            f(0.),
            f(1.),
            f(1.),
            f(1.),
            f(0.),
            s("plane"),
            s("null_surface"),
        ]);
        curve.values.extend(uv(0.5));
        curve.values.extend([s("nullbs"), b(16), b(11), b(11)]);
        let mut on_curve = RawEntity::new(3, "pcurve");
        on_curve.values = vec![
            AcisValue::Reference(crate::NULL_REF),
            int(-1),
            AcisValue::Reference(crate::EntityRef(2)),
            f(0.),
            f(0.),
        ];
        let mut shifted = on_curve.clone();
        shifted.index = 4;
        shifted.values[3] = f(1.);
        let model = AcisModel::new(
            AcisMetadata {
                save_version: Some(22300.into()),
                ..Default::default()
            },
            [own, alias, curve, on_curve, shifted]
                .into_iter()
                .map(Entity::Raw)
                .collect(),
            vec![],
        )
        .unwrap();
        let resolver = SubtypeResolver::new(Arc::new(model));
        let start = |i: usize| resolver.pcurve_fit(i).unwrap().unwrap().uv.curve.poles[0].x;
        assert_eq!((start(0), start(1), start(3)), (0.1, 0.1, 0.5));
        assert!(resolver.pcurve_fit(0).unwrap().unwrap().reversed);
        assert!(!resolver.pcurve_fit(1).unwrap().unwrap().reversed);
        assert!(resolver.pcurve_fit(3).unwrap().unwrap().reversed);
        assert_eq!(
            resolver.pcurve_fit(4).unwrap().unwrap().uv_shift,
            [1.0, 0.0]
        );
        assert!(resolver.pcurve_fit(2).unwrap().is_none());
    }
}
