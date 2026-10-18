//! Shared ACIS model owned by Rust. No Python, CadQuery, or container parser dependency.
//!
//! Adapters must decode an exact source dialect and remap references before calling
//! [`AcisModel::new`]. Validation proves reference closure, not B-rep validity or
//! compatibility with a particular Inventor/ASM version.
#![doc = include_str!("../README.md")]

pub mod approximation;
mod asm_nurbs;
pub mod authoring;
pub mod encode;
pub mod entities;
pub mod geometry;
pub mod nurbs;
pub mod pcurve;
pub mod sab;
pub mod subtypes;
pub mod supported_curve;
pub mod tolerant;
pub use entities::*;
pub use geometry::{GeometryError, Vec3};
pub use num_bigint::BigInt;
use std::{error::Error, fmt};

/// ASM save versions whose explicit spline, pcurve, subtype and tolerant layouts are qualified.
/// 22300 is written by AutoCAD 2018+ in DWG (AC1032) modeler entities, with the same field layouts as 22700.
pub(crate) fn qualified_save_version(version: u32) -> bool {
    matches!(version, 22300 | 22700)
}

/// Same check on a model's metadata.
pub(crate) fn qualified_model(model: &AcisModel) -> bool {
    model
        .metadata()
        .save_version
        .as_ref()
        .and_then(|v| v.to_string().parse().ok())
        .is_some_and(qualified_save_version)
}

/// Embedded subtype versions saved inside qualified files (22800 and 23100 occur in AutoCAD 22300 files).
pub(crate) fn qualified_embedded_version(version: usize) -> bool {
    matches!(version, 22601 | 22800 | 23100)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcisContainer {
    Sat,
    Sab,
    AsmSab,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct AcisMetadata {
    /// Millimetres per source length unit; None is an absent source fact.
    pub units_mm: Option<f64>,
    /// Absolute tolerance in source units.
    pub resabs: Option<f64>,
    pub resnor: Option<f64>,
    pub container: Option<AcisContainer>,
    pub save_version: Option<BigInt>,
    pub product_id: Option<String>,
    pub modeler_version: Option<String>,
    pub creation_date: Option<String>,
    pub dialect: Option<String>,
}

/// Half-open byte range in a named byte domain. Inflated stream offsets are
/// distinct from offsets in the outer file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceSpan {
    pub source_id: String,
    pub start_offset: usize,
    pub end_offset: usize,
}

impl SourceSpan {
    pub fn validate(&self) -> Result<(), ModelError> {
        if self.source_id.is_empty() || self.end_offset < self.start_offset {
            return Err(ModelError::InvalidSourceSpan);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AcisDiagnostic {
    pub code: String,
    pub message: String,
    pub entity_index: Option<usize>,
    pub source: Option<SourceSpan>,
}

/// Model-local index, with -1 representing null. Source IDs are separate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityRef(pub i64);
pub const NULL_REF: EntityRef = EntityRef(-1);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CountedString {
    pub value: String,
    pub declared_length: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AcisValue {
    Reference(EntityRef),
    CountedString(CountedString),
    Integer(BigInt),
    Float(f64),
    String(String),
    Bytes(Vec<u8>),
}

/// Optional legacy attachment; no SAT record is required for binary sources.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SatRecord {
    pub entity_type: String,
    pub text: String,
    pub sequence_number: Option<BigInt>,
    pub start_offset: usize,
    pub end_offset: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RawEntity {
    pub index: usize,
    pub type_name: String,
    pub attributes: EntityRef,
    pub entity_id: Option<BigInt>,
    pub values: Vec<AcisValue>,
    pub record: Option<SatRecord>,
    pub source: Option<SourceSpan>,
    pub raw_data: Option<Vec<u8>>,
}

impl RawEntity {
    pub fn new(index: usize, type_name: impl Into<String>) -> Self {
        Self {
            index,
            type_name: type_name.into(),
            attributes: NULL_REF,
            entity_id: None,
            values: Vec::new(),
            record: None,
            source: None,
            raw_data: None,
        }
    }

    pub fn references(&self) -> Vec<EntityRef> {
        std::iter::once(self.attributes)
            .chain(self.values.iter().filter_map(|v| {
                if let AcisValue::Reference(reference) = v {
                    Some(*reference)
                } else {
                    None
                }
            }))
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ModelError {
    InvalidIdentity { position: usize, declared: usize },
    InvalidReference { index: i64, entity_count: usize },
    InvalidSourceSpan,
    InvalidDiagnosticTarget { index: usize, entity_count: usize },
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIdentity { position, declared } => {
                write!(f, "entity at index {position} declares index {declared}")
            }
            Self::InvalidReference {
                index,
                entity_count,
            } => write!(
                f,
                "ACIS reference ${index} is outside a model of {entity_count} entities"
            ),
            Self::InvalidSourceSpan => write!(f, "invalid source byte domain or range"),
            Self::InvalidDiagnosticTarget {
                index,
                entity_count,
            } => write!(
                f,
                "diagnostic target {index} is outside a model of {entity_count} entities"
            ),
        }
    }
}
impl Error for ModelError {}

pub fn resolve_index(
    reference: EntityRef,
    entity_count: usize,
) -> Result<Option<usize>, ModelError> {
    if reference == NULL_REF {
        return Ok(None);
    }
    if let Ok(index) = usize::try_from(reference.0) {
        if index < entity_count {
            return Ok(Some(index));
        }
    }
    Err(ModelError::InvalidReference {
        index: reference.0,
        entity_count,
    })
}

/// Immutable validated snapshot. Fields are private so closure cannot become stale.
#[derive(Debug, Clone, PartialEq)]
pub struct AcisModel {
    metadata: AcisMetadata,
    entities: Vec<Entity>,
    diagnostics: Vec<AcisDiagnostic>,
}

impl AcisModel {
    pub fn new(
        metadata: AcisMetadata,
        entities: Vec<Entity>,
        diagnostics: Vec<AcisDiagnostic>,
    ) -> Result<Self, ModelError> {
        for (position, entity) in entities.iter().enumerate() {
            if entity.index() != position {
                return Err(ModelError::InvalidIdentity {
                    position,
                    declared: entity.index(),
                });
            }
            for reference in entity.references() {
                resolve_index(reference, entities.len())?;
            }
            if let Some(source) = &entity.raw().source {
                source.validate()?;
            }
        }
        for diagnostic in &diagnostics {
            if let Some(source) = &diagnostic.source {
                source.validate()?;
            }
            if let Some(index) = diagnostic.entity_index {
                if index >= entities.len() {
                    return Err(ModelError::InvalidDiagnosticTarget {
                        index,
                        entity_count: entities.len(),
                    });
                }
            }
        }
        Ok(Self {
            metadata,
            entities,
            diagnostics,
        })
    }

    pub fn metadata(&self) -> &AcisMetadata {
        &self.metadata
    }
    pub fn entities(&self) -> &[Entity] {
        &self.entities
    }
    pub fn diagnostics(&self) -> &[AcisDiagnostic] {
        &self.diagnostics
    }
    pub fn len(&self) -> usize {
        self.entities.len()
    }
    pub fn is_empty(&self) -> bool {
        self.entities.is_empty()
    }
    pub fn resolve(&self, reference: EntityRef) -> Result<Option<&Entity>, ModelError> {
        Ok(resolve_index(reference, self.len())?.map(|index| &self.entities[index]))
    }
    pub fn bodies(&self) -> impl Iterator<Item = &BodyEntity> {
        self.entities.iter().filter_map(|entity| {
            if let Entity::Body(body) = entity {
                Some(body)
            } else {
                None
            }
        })
    }
    pub fn referenced_by(&self, reference: EntityRef) -> Result<Vec<usize>, ModelError> {
        resolve_index(reference, self.len())?;
        Ok(self
            .entities
            .iter()
            .filter(|entity| entity.references().contains(&reference))
            .map(Entity::index)
            .collect())
    }
}
