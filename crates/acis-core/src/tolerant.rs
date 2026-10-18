//! Partial views of the observed ASM 22700 tolerant topology layouts.
//!
//! The original entities remain raw. In particular, the saved scalar fields are
//! not a license to enlarge a geometric tolerance or discard inline geometry.
use crate::{AcisValue, CoedgeEntity, EdgeEntity, EntityRef, RawEntity, VertexEntity};

#[derive(Debug, Clone, PartialEq)]
pub struct TolerantVertex {
    pub vertex: VertexEntity,
    pub source_flag: i64,
    /// Three saved doubles, in source order. The first may be -1 (unspecified).
    /// The meanings of the two later ASM fields are not yet qualified.
    pub saved_scalars: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct TolerantEdge {
    pub edge: EdgeEntity,
    pub saved_scalar: f64,
    pub embedded_version: i64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TolerantCoedge {
    pub coedge: CoedgeEntity,
    pub parameter_interval: [f64; 2],
    /// Uninterpreted attachment; only the null attachment profile is admitted.
    pub attachment: EntityRef,
    pub inline_curve: Option<Box<crate::supported_curve::SupportedCurve>>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TolerantTopology {
    Vertex(TolerantVertex),
    Edge(TolerantEdge),
    Coedge(TolerantCoedge),
}

struct Reader<'a> {
    values: &'a [AcisValue],
    pos: usize,
}
impl Reader<'_> {
    fn take(&mut self) -> Result<&AcisValue, String> {
        let value = self
            .values
            .get(self.pos)
            .ok_or("truncated tolerant topology")?;
        self.pos += 1;
        Ok(value)
    }
    fn reference(&mut self) -> Result<EntityRef, String> {
        match self.take()? {
            AcisValue::Reference(r) => Ok(*r),
            _ => Err("expected topology reference".into()),
        }
    }
    fn int(&mut self) -> Result<i64, String> {
        match self.take()? {
            AcisValue::Integer(v) => v
                .to_string()
                .parse()
                .map_err(|_| "invalid topology integer".into()),
            _ => Err("expected topology integer".into()),
        }
    }
    fn zero(&mut self) -> Result<(), String> {
        if self.int()? == 0 {
            Ok(())
        } else {
            Err("unqualified topology extension".into())
        }
    }
    fn number(&mut self) -> Result<f64, String> {
        match self.take()? {
            AcisValue::Float(v) if v.is_finite() => Ok(*v),
            _ => Err("expected finite topology double".into()),
        }
    }
    fn boolean(&mut self) -> Result<bool, String> {
        match self.take()? {
            AcisValue::Bytes(v) if v == &[10] => Ok(true),
            AcisValue::Bytes(v) if v == &[11] => Ok(false),
            _ => Err("expected topology Boolean".into()),
        }
    }
    fn text(&mut self) -> Result<String, String> {
        match self.take()? {
            AcisValue::String(v) => Ok(v.clone()),
            _ => Err("expected topology string".into()),
        }
    }
}

/// Decode an additive partial view, without changing the model's entity kinds.
/// `None` means a different entity/version; `Err` means an unqualified layout.
pub fn decode(raw: &RawEntity, save_version: u32) -> Result<Option<TolerantTopology>, String> {
    if !crate::qualified_save_version(save_version) {
        return Ok(None);
    }
    let mut r = Reader {
        values: &raw.values,
        pos: 0,
    };
    // ASM 22700 ends vertex/edge records with a zero extension field; AutoCAD 22300 files do not.
    let trailer = save_version != 22300;
    let view = match raw.type_name.as_str() {
        "tvertex-vertex" => {
            let pattern = r.reference()?;
            let edge = r.reference()?;
            let source_flag = r.int()?;
            if !matches!(source_flag, 0 | 1) {
                return Err("unqualified vertex flag".into());
            }
            let point = r.reference()?;
            let saved_scalars = [r.number()?, r.number()?, r.number()?];
            // -1 marks an unspecified value (AutoCAD 22300 files also leave the second one unspecified).
            if saved_scalars[0] != -1.0 && saved_scalars[0] < 0.0
                || saved_scalars[1] != -1.0 && saved_scalars[1] < 0.0
                || saved_scalars[2] < 0.0
            {
                return Err("invalid saved vertex scalar".into());
            }
            if trailer {
                r.zero()?;
            }
            TolerantTopology::Vertex(TolerantVertex {
                vertex: VertexEntity {
                    raw: raw.clone(),
                    pattern,
                    edge,
                    point,
                },
                source_flag,
                saved_scalars,
            })
        }
        "tedge-edge" => {
            let edge = EdgeEntity {
                raw: raw.clone(),
                pattern: r.reference()?,
                start_vertex: r.reference()?,
                start_parameter: Some(r.number()?),
                end_vertex: r.reference()?,
                end_parameter: Some(r.number()?),
                coedge: r.reference()?,
                curve: r.reference()?,
                reversed: r.boolean()?,
                convexity: Some(r.text()?),
            };
            let saved_scalar = r.number()?;
            let embedded_version = r.int()?;
            // AutoCAD 22300 files save the file version here.
            let known_version = embedded_version == i64::from(save_version)
                || usize::try_from(embedded_version).is_ok_and(crate::qualified_embedded_version);
            if saved_scalar < 0.0 || !known_version {
                return Err("unqualified tolerant edge profile".into());
            }
            if trailer {
                r.zero()?;
            }
            TolerantTopology::Edge(TolerantEdge {
                edge,
                saved_scalar,
                embedded_version,
            })
        }
        "tcoedge-coedge" => {
            let pattern = r.reference()?;
            let next_coedge = r.reference()?;
            let previous_coedge = r.reference()?;
            let partner_coedge = r.reference()?;
            let edge = r.reference()?;
            let reversed = r.boolean()?;
            let loop_ref = r.reference()?;
            r.zero()?;
            let pcurve = r.reference()?;
            let parameter_interval = [r.number()?, r.number()?];
            if parameter_interval[0] > parameter_interval[1] {
                return Err("reversed tolerant coedge interval".into());
            }
            let attachment = r.reference()?;
            if attachment.0 != -1 {
                return Err("unqualified tolerant coedge attachment".into());
            }
            let inline_curve = match r.int()? {
                0 => {
                    if r.text()? != "null_curve" {
                        return Err("unqualified null coedge curve".into());
                    }
                    None
                }
                1 => {
                    if r.text()? != "intcurve" {
                        return Err("unqualified inline coedge curve".into());
                    }
                    let view = crate::supported_curve::decode(raw, r.pos, None)?;
                    if view.curve.knots.first() != Some(&parameter_interval[0])
                        || view.curve.knots.last() != Some(&parameter_interval[1])
                    {
                        return Err("inline curve and coedge intervals differ".into());
                    }
                    r.pos = view.value_end;
                    Some(Box::new(view))
                }
                _ => return Err("unqualified topology extension".into()),
            };
            r.zero()?;
            TolerantTopology::Coedge(TolerantCoedge {
                coedge: CoedgeEntity {
                    raw: raw.clone(),
                    pattern,
                    next_coedge,
                    previous_coedge,
                    partner_coedge,
                    edge,
                    reversed,
                    loop_ref,
                    pcurve,
                },
                parameter_interval,
                attachment,
                inline_curve,
            })
        }
        _ => return Ok(None),
    };
    if r.pos != r.values.len() {
        return Err("unconsumed tolerant topology fields".into());
    }
    Ok(Some(view))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn vertex_profile_is_bounded_and_lossless() {
        let mut raw = RawEntity::new(0, "tvertex-vertex");
        raw.values = vec![
            AcisValue::Reference(EntityRef(-1)),
            AcisValue::Reference(EntityRef(2)),
            AcisValue::Integer(1.into()),
            AcisValue::Reference(EntityRef(3)),
            AcisValue::Float(-1.0),
            AcisValue::Float(0.001),
            AcisValue::Float(0.002),
            AcisValue::Integer(0.into()),
        ];
        let Some(TolerantTopology::Vertex(v)) = decode(&raw, 22700).unwrap() else {
            panic!()
        };
        assert_eq!(v.vertex.raw, raw);
        assert_eq!(v.saved_scalars, [-1.0, 0.001, 0.002]);
        assert!(decode(&raw, 22600).unwrap().is_none());
        for len in 0..raw.values.len() {
            let mut truncated = raw.clone();
            truncated.values.truncate(len);
            assert!(decode(&truncated, 22700).is_err());
        }
        for bad in [-2.0, f64::NAN, f64::INFINITY] {
            let mut invalid = raw.clone();
            invalid.values[5] = AcisValue::Float(bad);
            assert!(decode(&invalid, 22700).is_err());
        }
        raw.values.push(AcisValue::Integer(0.into()));
        assert!(decode(&raw, 22700).is_err());
    }
    #[test]
    fn autocad_22300_profile_has_no_trailer_and_saves_its_own_version() {
        use AcisValue::{Bytes as B, Float as F, Integer as I, Reference as R, String as S};
        let mut vertex = RawEntity::new(0, "tvertex-vertex");
        vertex.values = vec![
            R(EntityRef(-1)),
            R(EntityRef(2)),
            I(0.into()),
            R(EntityRef(3)),
            F(-1.0),
            F(-1.0),
            F(2.7e-6),
        ];
        let Some(TolerantTopology::Vertex(v)) = decode(&vertex, 22300).unwrap() else {
            panic!()
        };
        assert_eq!(v.saved_scalars, [-1.0, -1.0, 2.7e-6]);
        assert!(
            decode(&vertex, 22700).is_err(),
            "22700 requires the trailing zero"
        );
        let mut edge = RawEntity::new(1, "tedge-edge");
        edge.values = vec![
            R(EntityRef(-1)),
            R(EntityRef(4)),
            F(-0.36),
            R(EntityRef(5)),
            F(0.0),
            R(EntityRef(6)),
            R(EntityRef(7)),
            B(vec![11]),
            S("unknown".into()),
            F(3.9e-4),
            I(22300.into()),
        ];
        let Some(TolerantTopology::Edge(e)) = decode(&edge, 22300).unwrap() else {
            panic!()
        };
        assert_eq!((e.saved_scalar, e.embedded_version), (3.9e-4, 22300));
        let mut trailer = edge.clone();
        trailer.values.push(I(0.into()));
        assert!(decode(&trailer, 22300).is_err());
        edge.values[10] = I(22301.into());
        assert!(decode(&edge, 22300).is_err());
    }
}
