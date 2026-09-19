use pyo3::prelude::*;
use spaceros_procgen_envs::envs::{Asset, AssetVariant, Assets, EnvironmentConfig, Scenario};

pub(crate) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let m = PyModule::new_bound(parent.py(), crate::macros::python_module_name!())?;

    m.add_class::<Asset>()?;
    m.add_class::<Assets>()?;
    m.add_class::<AssetVariant>()?;
    m.add_class::<EnvironmentConfig>()?;
    m.add_class::<Scenario>()?;

    crate::macros::python_add_submodule!(parent, m)
}
