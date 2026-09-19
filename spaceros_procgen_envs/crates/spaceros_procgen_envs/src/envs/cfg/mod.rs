mod assets;
mod config;
mod enums;

pub use assets::*;
pub use config::*;
pub use enums::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract() {
        figment::Jail::expect_with(|jail| {
            // Arrange
            let cfg_path = if cfg!(feature = "yaml") {
                jail.create_file(
                    "config.yaml",
                    r"
                    seed: 42
                    assets:
                      vehicle:
                        variant: dataset
                ",
                )?;
                Some("config.yaml")
            } else {
                None
            };
            jail.set_env("SPACEROS_DEMO_SCENARIO", "ORBIT");
            jail.set_env("SPACEROS_DEMO_ASSETS_TERRAIN_VARIANT", "PROCEDURAL");
            jail.set_env("SPACEROS_DEMO_DETAIL", "0.2");

            // Act
            let config = EnvironmentConfig::extract(cfg_path, Some("SPACEROS_DEMO_"), None)?;

            // Assert
            assert_eq!(
                config,
                EnvironmentConfig {
                    scenario: Scenario::Orbit,
                    assets: Assets {
                        vehicle: Asset {
                            variant: AssetVariant::Dataset,
                        },
                        terrain: Asset {
                            variant: AssetVariant::Procedural,
                        },
                        ..EnvironmentConfig::default().assets
                    },
                    seed: 42,
                    detail: 0.2,
                }
            );

            Ok(())
        });
    }
}
