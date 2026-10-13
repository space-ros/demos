use super::AssetVariant;
use pyo3::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[pyclass(eq, get_all, set_all)]
pub struct Assets {
    pub robot: Asset,
    pub object: Asset,
    pub terrain: Asset,
    pub vehicle: Asset,
}

#[pymethods]
impl Assets {
    fn __repr__(&self) -> String {
        format!("{self:?}")
    }

    #[new]
    fn new(robot: Asset, object: Asset, terrain: Asset, vehicle: Asset) -> Self {
        Self {
            robot,
            object,
            terrain,
            vehicle,
        }
    }
}

#[derive(Deserialize, Serialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[pyclass(eq, get_all, set_all)]
pub struct Asset {
    pub variant: AssetVariant,
}

#[pymethods]
impl Asset {
    fn __repr__(&self) -> String {
        format!("{self:?}")
    }

    #[new]
    fn new(variant: AssetVariant) -> Self {
        Self { variant }
    }
}
