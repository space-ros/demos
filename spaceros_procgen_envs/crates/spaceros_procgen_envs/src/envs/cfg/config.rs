use super::{Asset, AssetVariant, Assets, Scenario};
use crate::Result;
#[cfg(any(feature = "json", feature = "toml", feature = "yaml"))]
use figment::providers::Format;
use figment::Figment;
use pyo3::prelude::*;
use serde::{Deserialize, Serialize};
#[cfg(any(feature = "json", feature = "toml", feature = "yaml"))]
use std::io::Write;

#[derive(Deserialize, Serialize, Debug, Clone, Copy, PartialEq)]
#[pyclass(eq, get_all, set_all)]
pub struct EnvironmentConfig {
    pub scenario: Scenario,
    pub assets: Assets,
    pub seed: u64,
    pub detail: f32,
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        Self {
            scenario: Scenario::Moon,
            assets: Assets {
                robot: Asset {
                    variant: AssetVariant::Dataset,
                },
                object: Asset {
                    variant: AssetVariant::Procedural,
                },
                terrain: Asset {
                    variant: AssetVariant::Procedural,
                },
                vehicle: Asset {
                    variant: AssetVariant::Dataset,
                },
            },
            seed: 0,
            detail: 1.0,
        }
    }
}

impl EnvironmentConfig {
    pub fn extract(
        cfg_path: Option<impl AsRef<std::path::Path>>,
        env_prefix: Option<&str>,
        other: Option<Self>,
    ) -> Result<Self> {
        let mut figment = Figment::new();

        // 1. (Optional) Load configuration from file
        if let Some(path) = cfg_path {
            let path = path.as_ref();
            if !path.exists() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("File not found: {}", path.display()),
                )
                .into());
            }
            match path.extension().and_then(|ext| ext.to_str()) {
                #[cfg(feature = "json")]
                Some("json") => figment = figment.merge(figment::providers::Json::file(path)),
                #[cfg(feature = "toml")]
                Some("toml") => figment = figment.merge(figment::providers::Toml::file(path)),
                #[cfg(feature = "yaml")]
                Some("yaml" | "yml") => {
                    figment = figment.merge(figment::providers::Yaml::file(path));
                }
                Some(_) => {
                    return Err(figment::Error::from(format!(
                        "Unsupported file extension: {} (supported=[{}])",
                        path.display(),
                        crate::utils::SUPPORTED_FILE_EXTENSIONS.join(", "),
                    ))
                    .into())
                }
                None => {
                    return Err(figment::Error::from(format!(
                        "Missing file extension: {}",
                        path.display()
                    ))
                    .into())
                }
            }
        }

        // 2. (Optional) Load configuration from environment variables
        if let Some(env_prefix) = env_prefix {
            figment = figment.merge(figment::providers::Env::prefixed(env_prefix).split('_'));
        }

        // 3. (Optional) Load configuration from other sources
        if let Some(other) = other {
            figment = figment.merge(figment::providers::Serialized::defaults(other));
        }

        // Finally, apply default values for missing fields
        figment = figment.join(figment::providers::Serialized::defaults(
            EnvironmentConfig::default(),
        ));

        Ok(figment.extract()?)
    }

    pub fn write(&self, path: impl AsRef<std::path::Path>) -> Result<()> {
        let path = path.as_ref();

        // Return error if none or if not supported extension
        match path.extension().and_then(|ext| ext.to_str()) {
            #[cfg(feature = "json")]
            Some("json") => {
                let file = std::fs::File::create(path)?;
                let mut writer = std::io::BufWriter::new(file);
                serde_json::to_writer_pretty(&mut writer, self).map_err(|e| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                })?;
                writer.flush()?;
                Ok(())
            }
            #[cfg(feature = "toml")]
            Some("toml") => {
                let mut file = std::fs::File::create(path)?;
                let content = toml::to_string_pretty(self).map_err(|e| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                })?;
                file.write_all(content.as_bytes())?;
                Ok(())
            }
            #[cfg(feature = "yaml")]
            Some("yaml" | "yml") => {
                let file = std::fs::File::create(path)?;
                let mut writer = std::io::BufWriter::new(file);
                serde_yaml::to_writer(&mut writer, self).map_err(|e| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                })?;
                writer.flush()?;
                Ok(())
            }
            Some(_) | None => Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "Unsupported file extension: {} (supported=[{}])",
                    path.display(),
                    crate::utils::SUPPORTED_FILE_EXTENSIONS.join(", "),
                ),
            )
            .into()),
        }
    }
}

#[pymethods]
impl EnvironmentConfig {
    fn __repr__(&self) -> String {
        format!("{self:?}")
    }

    #[new]
    fn new(scenario: Scenario, assets: Assets, seed: u64, detail: f32) -> Self {
        Self {
            scenario,
            assets,
            seed,
            detail,
        }
    }

    #[staticmethod]
    #[pyo3(
        name = "extract",
        signature = (cfg_path=None, env_prefix=Some("SPACEROS_DEMO_"), other=None),
    )]
    fn py_extract(
        cfg_path: Option<&str>,
        env_prefix: Option<&str>,
        other: Option<Self>,
    ) -> PyResult<Self> {
        Ok(Self::extract(cfg_path, env_prefix, other)?)
    }

    #[pyo3(name = "write")]
    fn py_write(&self, path: &str) -> PyResult<()> {
        Ok(self.write(path)?)
    }

    #[cfg(any(feature = "json", feature = "toml", feature = "yaml"))]
    fn __reduce__(&self) -> PyResult<(PyObject, PyObject)> {
        Python::with_gil(|py| {
            py.run_bound("import spaceros_procgen_envs", None, None)
                .unwrap();
            let deserialize = py
                .eval_bound(
                    "spaceros_procgen_envs._rs.envs.EnvironmentConfig._deserialize",
                    None,
                    None,
                )
                .unwrap();

            #[cfg(feature = "json")]
            let data = serde_json::to_vec(self).unwrap();
            #[cfg(all(feature = "yaml", not(feature = "json")))]
            let data = serde_yaml::to_string(self).unwrap().as_bytes().to_vec();
            #[cfg(all(feature = "toml", not(any(feature = "json", feature = "yaml"))))]
            let data = toml::to_string(self).unwrap().as_bytes().to_vec();

            Ok((deserialize.to_object(py), (data,).to_object(py)))
        })
    }

    #[staticmethod]
    #[pyo3(name = "_deserialize")]
    #[cfg(any(feature = "json", feature = "toml", feature = "yaml"))]
    fn deserialize(data: Vec<u8>) -> PyResult<Self> {
        #[cfg(feature = "json")]
        {
            Ok(serde_json::from_slice(&data).unwrap())
        }
        #[cfg(all(feature = "yaml", not(feature = "json")))]
        {
            Ok(serde_yaml::from_slice(&data).unwrap())
        }
        #[cfg(all(feature = "toml", not(any(feature = "json", feature = "yaml"))))]
        {
            Ok(toml::from_str(std::str::from_utf8(&data).unwrap()).unwrap())
        }
    }
}
