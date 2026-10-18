//! Bounded SAB/ASM active-record decoder. No SAT serialization is involved.
//! Tag grammar: ezdxf (MIT) and cadmpeg format notes (CC-BY-4.0), cited in
//! docs/sab-support.md. History remains an explicit opaque source attachment.
use crate::*;
use std::{error::Error, fmt};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SabError {
    pub offset: usize,
    pub message: String,
}
impl fmt::Display for SabError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SAB byte {}: {}", self.offset, self.message)
    }
}
impl Error for SabError {}
type Result<T> = std::result::Result<T, SabError>;
fn error(offset: usize, message: impl Into<String>) -> SabError {
    SabError {
        offset,
        message: message.into(),
    }
}

#[derive(Debug, Clone)]
pub struct SabLimits {
    pub max_bytes: usize,
    pub max_records: usize,
    pub max_tokens: usize,
    pub max_string_bytes: usize,
    pub max_depth: usize,
}
impl Default for SabLimits {
    fn default() -> Self {
        Self {
            max_bytes: 64 * 1024 * 1024,
            max_records: 500_000,
            max_tokens: 4_000_000,
            max_string_bytes: 4 * 1024 * 1024,
            max_depth: 64,
        }
    }
}
#[derive(Debug, Clone, PartialEq)]
pub struct SabHeader {
    pub container: AcisContainer,
    pub integer_width: u8,
    pub save_version: u32,
    pub record_count: u64,
    pub entity_count: u64,
    pub flags: u64,
    pub product_id: String,
    pub modeler_version: String,
    pub creation_date: String,
    pub scale: f64,
    pub resabs: f64,
    pub resnor: f64,
    pub data_offset: usize,
}
#[derive(Debug, Clone)]
pub struct SabDocument {
    pub header: SabHeader,
    pub model: AcisModel,
    pub history: Option<SourceSpan>,
}

#[derive(Debug, Clone)]
enum Value {
    Int(i64),
    Float(f64),
    Ref(EntityRef),
    Text(String),
    Bool(bool),
    Vec3(Vec3),
    Opaque,
}
#[derive(Debug, Clone)]
struct Token {
    value: Value,
    start: usize,
    end: usize,
}
struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
    width: usize,
    limits: &'a SabLimits,
    tokens: usize,
}
impl<'a> Cursor<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self
            .pos
            .checked_add(n)
            .ok_or_else(|| error(self.pos, "length overflow"))?;
        let data = self
            .data
            .get(self.pos..end)
            .ok_or_else(|| error(self.pos, "truncated payload"))?;
        self.pos = end;
        Ok(data)
    }
    fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }
    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn word(&mut self) -> Result<i64> {
        if self.width == 8 {
            Ok(i64::from_le_bytes(self.take(8)?.try_into().unwrap()))
        } else {
            Ok(self.u32()? as i32 as i64)
        }
    }
    fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn text(&mut self, length: usize) -> Result<String> {
        if length > self.limits.max_string_bytes {
            return Err(error(self.pos, "string exceeds limit"));
        }
        let start = self.pos;
        String::from_utf8(self.take(length)?.to_vec()).map_err(|_| error(start, "invalid UTF-8"))
    }
    fn name(&mut self) -> Result<String> {
        let mut names = Vec::new();
        loop {
            let tag = self.u8()?;
            if tag != 13 && tag != 14 {
                return Err(error(self.pos - 1, "expected record identifier"));
            }
            let len = self.u8()? as usize;
            names.push(self.text(len)?);
            if names.len() > 32 {
                return Err(error(self.pos, "identifier lineage exceeds limit"));
            }
            if tag == 13 {
                return Ok(names.join("-"));
            }
        }
    }
    fn token(&mut self) -> Result<(u8, Token)> {
        self.tokens += 1;
        if self.tokens > self.limits.max_tokens {
            return Err(error(self.pos, "token limit exceeded"));
        }
        let start = self.pos;
        let tag = self.u8()?;
        let value = match tag {
            2 => Value::Int(self.u8()? as i64),
            3 => Value::Int(i16::from_le_bytes(self.take(2)?.try_into().unwrap()) as i64),
            4 | 21 => Value::Int(self.word()?),
            5 => Value::Float(f32::from_le_bytes(self.take(4)?.try_into().unwrap()) as f64),
            6 => Value::Float(self.f64()?),
            7 | 8 | 9 | 18 => {
                let len = match tag {
                    7 => self.u8()? as i64,
                    8 => u16::from_le_bytes(self.take(2)?.try_into().unwrap()) as i64,
                    _ => self.word()?,
                };
                let len =
                    usize::try_from(len).map_err(|_| error(start, "negative string length"))?;
                Value::Text(self.text(len)?)
            }
            10 => Value::Bool(true),
            11 => Value::Bool(false),
            12 => Value::Ref(EntityRef(self.word()?)),
            13 | 14 => {
                let len = self.u8()? as usize;
                Value::Text(self.text(len)?)
            }
            15..=17 => Value::Opaque,
            19 | 20 => Value::Vec3((self.f64()?, self.f64()?, self.f64()?).into()),
            22 => {
                self.take(16)?;
                Value::Opaque
            }
            23 => {
                self.take(8)?;
                Value::Opaque
            }
            _ => return Err(error(start, format!("unknown tag 0x{tag:02x}"))),
        };
        Ok((
            tag,
            Token {
                value,
                start,
                end: self.pos,
            },
        ))
    }
    fn header_string(&mut self) -> Result<String> {
        let (tag, t) = self.token()?;
        match (tag, t.value) {
            (7, Value::Text(s)) => Ok(s),
            _ => Err(error(t.start, "expected header string tag 7")),
        }
    }
    fn header_float(&mut self) -> Result<f64> {
        let (tag, t) = self.token()?;
        match (tag, t.value) {
            (6, Value::Float(v)) if v.is_finite() => Ok(v),
            _ => Err(error(t.start, "expected finite header double")),
        }
    }
}

/// Read only explicitly admitted save formats; individual entity schemas must
/// also consume their complete field sequence. Unknown entities remain raw.
pub fn parse_sab(data: &[u8], source_id: &str, limits: &SabLimits) -> Result<SabDocument> {
    if data.len() > limits.max_bytes {
        return Err(error(0, "input exceeds byte limit"));
    }
    if source_id.is_empty() {
        return Err(error(0, "source domain must not be empty"));
    }
    let (container, width) = if data.starts_with(b"ACIS BinaryFile") {
        (AcisContainer::Sab, 4)
    } else if data.starts_with(b"ASM BinaryFile4") {
        (AcisContainer::AsmSab, 4)
    } else if data.starts_with(b"ASM BinaryFile8") {
        (AcisContainer::AsmSab, 8)
    } else {
        return Err(error(0, "unrecognized binary signature"));
    };
    let mut c = Cursor {
        data,
        pos: 15,
        width,
        limits,
        tokens: 0,
    };
    let version = c.word()?;
    let allowed = match container {
        AcisContainer::Sab => version == 21800,
        AcisContainer::AsmSab => matches!(
            version,
            22000 | 22300 | 22600 | 22700 | 22900 | 23100 | 23200
        ),
        _ => false,
    };
    if !allowed {
        return Err(error(15, format!("unsupported SAB save version {version}")));
    }
    let record_count = c.word()?;
    let entity_count = c.word()?;
    let flags = c.word()?;
    if record_count < 0 || entity_count < 0 || !(0..=255).contains(&flags) {
        return Err(error(15, "invalid header counts/flags"));
    }
    let product_id = c.header_string()?;
    let modeler_version = c.header_string()?;
    let creation_date = c.header_string()?;
    let scale = c.header_float()?;
    let resabs = c.header_float()?;
    let resnor = c.header_float()?;
    let header = SabHeader {
        container,
        integer_width: width as u8,
        save_version: version as u32,
        record_count: record_count as u64,
        entity_count: entity_count as u64,
        flags: flags as u64,
        product_id,
        modeler_version,
        creation_date,
        scale,
        resabs,
        resnor,
        data_offset: c.pos,
    };
    let mut entities: Vec<Entity> = Vec::new();
    let mut diagnostics = Vec::new();
    let mut history = None;
    let mut ended = false;
    let mut opaque_count = 0;
    while c.pos < data.len() {
        let start = c.pos;
        let name = c.name()?;
        if name == "End-of-ASM-data" || name == "End-of-ACIS-data" {
            if c.data.get(c.pos) == Some(&17) {
                c.pos += 1;
            }
            if c.pos != data.len() {
                return Err(error(c.pos, "data after end marker"));
            }
            ended = true;
            break;
        }
        if entities.len() >= limits.max_records {
            return Err(error(start, "record limit exceeded"));
        }
        if name == "delta_state"
            || name == "Begin-of-ASM-History-Data"
            || name == "Begin-of-ACIS-History-Data"
        {
            if flags & 1 == 0 {
                return Err(error(start, "history without header flag"));
            }
            let source = SourceSpan {
                source_id: source_id.into(),
                start_offset: start,
                end_offset: data.len(),
            };
            let mut raw = RawEntity::new(entities.len(), "__opaque_sab_history__");
            raw.source = Some(source.clone());
            raw.raw_data = Some(data[start..].to_vec());
            diagnostics.push(AcisDiagnostic {
                code: "sab.history_opaque".into(),
                message:
                    "History suffix retained; only the preceding solved record table is decoded"
                        .into(),
                entity_index: Some(raw.index),
                source: Some(source.clone()),
            });
            for entity in &entities {
                for reference in entity.references() {
                    resolve_index(reference, entities.len())
                        .map_err(|e| error(start, e.to_string()))?;
                }
            }
            entities.push(Entity::Raw(raw));
            history = Some(source);
            ended = true;
            break;
        }
        let mut tokens = Vec::new();
        let mut depth = 0usize;
        loop {
            let (tag, t) = c.token()?;
            match tag {
                15 => {
                    depth += 1;
                    if depth > limits.max_depth {
                        return Err(error(t.start, "subtype depth limit exceeded"));
                    }
                }
                16 => {
                    depth = depth
                        .checked_sub(1)
                        .ok_or_else(|| error(t.start, "unmatched subtype close"))?;
                }
                17 => {
                    if depth != 0 {
                        return Err(error(t.start, "record ends inside subtype"));
                    }
                    break;
                }
                _ => {}
            }
            tokens.push(t);
        }
        let index = entities.len();
        let mut reader = Reader {
            tokens: &tokens,
            pos: 0,
            offset: start,
        };
        let attributes = reader.reference()?;
        let entity_id = reader.integer()?;
        let source = SourceSpan {
            source_id: source_id.into(),
            start_offset: start,
            end_offset: c.pos,
        };
        let values = tokens[2..]
            .iter()
            .map(|t| match &t.value {
                Value::Ref(v) => AcisValue::Reference(*v),
                Value::Int(v) => AcisValue::Integer((*v).into()),
                Value::Float(v) => AcisValue::Float(*v),
                Value::Text(v) => AcisValue::String(v.clone()),
                _ => AcisValue::Bytes(data[t.start..t.end].to_vec()),
            })
            .collect();
        let raw = RawEntity {
            index,
            type_name: name,
            attributes,
            entity_id: Some(entity_id.into()),
            values,
            record: None,
            source: Some(source.clone()),
            raw_data: Some(data[start..c.pos].to_vec()),
        };
        let entity = match crate::asm_nurbs::decode(&raw, header.save_version)
            .map_err(|message| error(start, message))
            .and_then(|decoded| match decoded {
                Some(entity) => Ok(Some(entity)),
                None => decode(&mut reader, raw.clone()),
            }) {
            Ok(Some(entity)) => entity,
            Ok(None) => {
                opaque_count += 1;
                if matches!(
                    raw.type_name.as_str(),
                    "spline-surface"
                        | "intcurve-curve"
                        | "pcurve"
                        | "tolerant-edge"
                        | "tolerant-vertex"
                        | "tolerant-coedge"
                        | "tvertex-vertex"
                        | "tedge-edge"
                        | "tcoedge-coedge"
                ) {
                    let subtype = raw
                        .values
                        .iter()
                        .find_map(|v| match v {
                            AcisValue::String(s) => Some(s.as_str()),
                            _ => None,
                        })
                        .unwrap_or("no named subtype");
                    diagnostics.push(AcisDiagnostic {
                        code: "sab.geometry_subtype_unsupported".into(),
                        message: format!(
                            "{}: {subtype}; source subtype/chart/topology is not qualified",
                            raw.type_name
                        ),
                        entity_index: Some(index),
                        source: Some(source.clone()),
                    });
                }
                Entity::Raw(raw)
            }
            Err(e) => {
                diagnostics.push(AcisDiagnostic {
                    code: "sab.entity_schema_unsupported".into(),
                    message: e.to_string(),
                    entity_index: Some(index),
                    source: Some(source),
                });
                Entity::Raw(raw)
            }
        };
        entities.push(entity);
    }
    if !ended {
        return Err(error(c.pos, "missing end marker"));
    }
    if entities.first().map(|e| e.raw().type_name.as_str()) != Some("asmheader") {
        return Err(error(header.data_offset, "expected asmheader record zero"));
    }
    if opaque_count > 0 {
        diagnostics.push(AcisDiagnostic {
            code: "sab.raw_entities".into(),
            message: format!("{opaque_count} records retained without semantic decoding"),
            entity_index: None,
            source: None,
        });
    }
    let metadata = AcisMetadata {
        units_mm: Some(scale),
        resabs: Some(resabs),
        resnor: Some(resnor),
        container: Some(container),
        save_version: Some(version.into()),
        product_id: Some(header.product_id.clone()),
        modeler_version: Some(header.modeler_version.clone()),
        creation_date: Some(header.creation_date.clone()),
        dialect: Some(format!("sab/{version}/{}-byte", width)),
    };
    let model =
        AcisModel::new(metadata, entities, diagnostics).map_err(|e| error(c.pos, e.to_string()))?;
    Ok(SabDocument {
        header,
        model,
        history,
    })
}

struct Reader<'a> {
    tokens: &'a [Token],
    pos: usize,
    offset: usize,
}
impl Reader<'_> {
    fn next(&mut self) -> Result<&Value> {
        let t = self
            .tokens
            .get(self.pos)
            .ok_or_else(|| error(self.offset, "missing entity field"))?;
        self.pos += 1;
        Ok(&t.value)
    }
    fn reference(&mut self) -> Result<EntityRef> {
        match self.next()? {
            Value::Ref(v) => Ok(*v),
            _ => Err(error(self.offset, "expected reference")),
        }
    }
    fn integer(&mut self) -> Result<i64> {
        match self.next()? {
            Value::Int(v) => Ok(*v),
            _ => Err(error(self.offset, "expected integer")),
        }
    }
    fn number(&mut self) -> Result<f64> {
        match self.next()? {
            Value::Float(v) if v.is_finite() => Ok(*v),
            _ => Err(error(self.offset, "expected finite double")),
        }
    }
    /// A finite double, or NaN for a value the source does not save (SAT before 500 has no edge parameters).
    fn optional_number(&mut self) -> Result<Option<f64>> {
        match self.next()? {
            Value::Float(v) if v.is_nan() => Ok(None),
            Value::Float(v) if v.is_finite() => Ok(Some(*v)),
            _ => Err(error(self.offset, "expected finite double or NaN")),
        }
    }
    fn vector(&mut self) -> Result<Vec3> {
        match self.next()? {
            Value::Vec3(v) if v.x.is_finite() && v.y.is_finite() && v.z.is_finite() => Ok(*v),
            _ => Err(error(self.offset, "expected finite vector")),
        }
    }
    fn boolean(&mut self) -> Result<bool> {
        match self.next()? {
            Value::Bool(v) => Ok(*v),
            _ => Err(error(self.offset, "expected Boolean")),
        }
    }
    fn text(&mut self) -> Result<String> {
        match self.next()? {
            Value::Text(v) => Ok(v.clone()),
            _ => Err(error(self.offset, "expected string")),
        }
    }
    fn bound(&mut self) -> Result<Option<f64>> {
        if self.boolean()? {
            Ok(Some(self.number()?))
        } else {
            Ok(None)
        }
    }
    fn range(&mut self) -> Result<Option<ParameterRange>> {
        Ok(Some(ParameterRange {
            lower: self.bound()?,
            upper: self.bound()?,
        }))
    }
    fn finish(&self) -> Result<()> {
        if self.pos == self.tokens.len() {
            Ok(())
        } else {
            Err(error(
                self.offset,
                format!("{} unconsumed entity fields", self.tokens.len() - self.pos),
            ))
        }
    }
}
fn decode(r: &mut Reader<'_>, raw: RawEntity) -> Result<Option<Entity>> {
    let name = raw.type_name.clone();
    let entity = match name.as_str() {
        "body" => Entity::Body(BodyEntity {
            raw,
            pattern: r.reference()?,
            lump: r.reference()?,
            wire: r.reference()?,
            transform: r.reference()?,
        }),
        "lump" => Entity::Lump(LumpEntity {
            raw,
            pattern: r.reference()?,
            next_lump: r.reference()?,
            shell: r.reference()?,
            body: r.reference()?,
        }),
        "shell" => Entity::Shell(ShellEntity {
            raw,
            pattern: r.reference()?,
            next_shell: r.reference()?,
            subshell: r.reference()?,
            face: r.reference()?,
            wire: r.reference()?,
            lump: r.reference()?,
        }),
        "face" => {
            let pattern = r.reference()?;
            let next_face = r.reference()?;
            let loop_ref = r.reference()?;
            let shell = r.reference()?;
            let subshell = r.reference()?;
            let surface = r.reference()?;
            let reversed = r.boolean()?;
            let double_sided = r.boolean()?;
            let containment_in = if double_sided {
                Some(r.boolean()?)
            } else {
                None
            };
            Entity::Face(FaceEntity {
                raw,
                pattern,
                next_face,
                loop_ref,
                shell,
                subshell,
                surface,
                reversed,
                double_sided,
                containment_in,
            })
        }
        "loop" => Entity::Loop(LoopEntity {
            raw,
            pattern: r.reference()?,
            next_loop: r.reference()?,
            coedge: r.reference()?,
            face: r.reference()?,
        }),
        "coedge" => {
            let pattern = r.reference()?;
            let next_coedge = r.reference()?;
            let previous_coedge = r.reference()?;
            let partner_coedge = r.reference()?;
            let edge = r.reference()?;
            let reversed = r.boolean()?;
            let loop_ref = r.reference()?;
            if r.integer()? != 0 {
                return Err(error(r.offset, "unsupported coedge selector"));
            }
            let pcurve = r.reference()?;
            Entity::Coedge(CoedgeEntity {
                raw,
                pattern,
                next_coedge,
                previous_coedge,
                partner_coedge,
                edge,
                reversed,
                loop_ref,
                pcurve,
            })
        }
        "edge" => Entity::Edge(EdgeEntity {
            raw,
            pattern: r.reference()?,
            start_vertex: r.reference()?,
            start_parameter: r.optional_number()?,
            end_vertex: r.reference()?,
            end_parameter: r.optional_number()?,
            coedge: r.reference()?,
            curve: r.reference()?,
            reversed: r.boolean()?,
            convexity: Some(r.text()?),
        }),
        "vertex" => {
            let pattern = r.reference()?;
            let edge = r.reference()?;
            let _source_vertex_flag = r.integer()?;
            let point = r.reference()?;
            Entity::Vertex(VertexEntity {
                raw,
                pattern,
                edge,
                point,
            })
        }
        "point" => Entity::Point(PointEntity {
            raw,
            pattern: r.reference()?,
            location: r.vector()?,
        }),
        "straight-curve" | "straight" => Entity::StraightCurve(StraightCurveEntity {
            raw,
            pattern: r.reference()?,
            origin: r.vector()?,
            direction: r.vector()?,
            parameter_range: r.range()?,
        }),
        "ellipse-curve" | "ellipse" => Entity::EllipseCurve(EllipseCurveEntity {
            raw,
            pattern: r.reference()?,
            center: r.vector()?,
            normal: r.vector()?,
            major_axis: r.vector()?,
            ratio: r.number()?,
            parameter_range: r.range()?,
        }),
        "plane-surface" | "plane" => Entity::PlaneSurface(PlaneSurfaceEntity {
            raw,
            pattern: r.reference()?,
            origin: r.vector()?,
            normal: r.vector()?,
            u_direction: r.vector()?,
            reverse_v: r.boolean()?,
            u_range: r.range()?,
            v_range: r.range()?,
        }),
        "cone-surface" | "cone" => {
            let pattern = r.reference()?;
            let center = r.vector()?;
            let axis = r.vector()?;
            let major_axis = r.vector()?;
            let ratio = r.number()?;
            let profile_range = r.range()?;
            let sin_half_angle = r.number()?;
            let cos_half_angle = r.number()?;
            let parameter_scale = r.number()?;
            let reversed = r.boolean()?;
            let u_range = r.range()?;
            let v_range = r.range()?;
            let radius = major_axis.magnitude();
            if radius <= 0.0
                || parameter_scale <= 0.0
                || ratio == 0.0
                || cos_half_angle.abs() <= 1e-12
                || (sin_half_angle * sin_half_angle + cos_half_angle * cos_half_angle - 1.0).abs()
                    > 1e-9
            {
                return Err(error(r.offset, "invalid cone frame/angle/scale"));
            }
            Entity::ConeSurface(ConeSurfaceEntity {
                raw,
                pattern,
                center,
                axis,
                major_axis,
                ratio,
                profile_range,
                sin_half_angle,
                cos_half_angle,
                reference_radius: radius,
                parameter_scale,
                reversed,
                u_range,
                v_range,
            })
        }
        "sphere-surface" | "sphere" => Entity::SphereSurface(SphereSurfaceEntity {
            raw,
            pattern: r.reference()?,
            center: r.vector()?,
            radius: r.number()?,
            u_direction: r.vector()?,
            pole: r.vector()?,
            reversed: r.boolean()?,
            u_range: r.range()?,
            v_range: r.range()?,
        }),
        "torus-surface" | "torus" => Entity::TorusSurface(TorusSurfaceEntity {
            raw,
            pattern: r.reference()?,
            center: r.vector()?,
            axis: r.vector()?,
            major_radius: r.number()?,
            minor_radius: r.number()?,
            u_direction: r.vector()?,
            reversed: r.boolean()?,
            u_range: r.range()?,
            v_range: r.range()?,
        }),
        "transform" if !matches!(r.tokens.get(r.pos).map(|t| &t.value), Some(Value::Text(_))) => {
            // Binary form: three matrix rows and the translation as vectors, scale, rotate/reflect/shear.
            let mut matrix_values = [0.0; 12];
            for row in 0..4 {
                let v = r.vector()?;
                matrix_values[row * 3..row * 3 + 3].copy_from_slice(&[v.x, v.y, v.z]);
            }
            Entity::Transform(TransformEntity {
                raw,
                matrix_values,
                scale: r.number()?,
                rotated: r.boolean()?,
                reflected: r.boolean()?,
                sheared: r.boolean()?,
            })
        }
        "transform" => {
            let text = r.text()?;
            let words = text.split_whitespace().collect::<Vec<_>>();
            if words.len() != 16 {
                return Err(error(r.offset, "transform requires 16 fields"));
            }
            let numbers = words[..13]
                .iter()
                .map(|s| {
                    s.parse::<f64>()
                        .map_err(|_| error(r.offset, "invalid transform number"))
                })
                .collect::<Result<Vec<_>>>()?;
            if !numbers.iter().all(|x| x.is_finite()) {
                return Err(error(r.offset, "nonfinite transform"));
            }
            let flag = |i, a, b| {
                if words[i] == a {
                    Ok(true)
                } else if words[i] == b {
                    Ok(false)
                } else {
                    Err(error(r.offset, "invalid transform flag"))
                }
            };
            Entity::Transform(TransformEntity {
                raw,
                matrix_values: numbers[..12].try_into().unwrap(),
                scale: numbers[12],
                rotated: flag(13, "rotate", "no_rotate")?,
                reflected: flag(14, "reflect", "no_reflect")?,
                sheared: flag(15, "shear", "no_shear")?,
            })
        }
        _ => return Ok(None),
    };
    r.finish()?;
    Ok(Some(entity))
}

#[cfg(test)]
#[path = "sab_tests.rs"]
mod tests;
