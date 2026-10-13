use pyo3::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[pyclass(frozen, eq, eq_int, hash, rename_all = "SCREAMING_SNAKE_CASE")]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Scenario {
    #[serde(alias = "asteroid")]
    Asteroid,
    #[serde(alias = "earth", alias = "TERRESTRIAL", alias = "terrestrial")]
    Earth,
    #[serde(alias = "mars", alias = "MARTIAN", alias = "martian")]
    Mars,
    #[default]
    #[serde(alias = "moon", alias = "LUNAR", alias = "lunar")]
    Moon,
    #[serde(alias = "orbit", alias = "ORBITAL", alias = "orbital")]
    Orbit,
}

impl Scenario {
    /// Magnitude of gravitational acceleration in m/s².
    ///
    /// # Assumptions
    ///
    /// - Asteroid: 50% gravitational acceleration of Ceres (largest body in the asteroid belt).
    /// - Orbit: No gravitational acceleration.
    #[must_use] pub fn gravity_magnitude(self) -> f64 {
        match self {
            Self::Asteroid => 0.14219,
            Self::Earth => 9.80665,
            Self::Mars => 3.72076,
            Self::Moon => 1.62496,
            Self::Orbit => 0.0,
        }
    }

    /// Difference between the maximum and minimum of gravitational acceleration in m/s².
    ///
    /// # Assumptions
    ///
    /// - Asteroid: Ceres is considered as the maximum (largest body in the asteroid belt).
    /// - Orbit: No gravitational acceleration.
    #[must_use] pub fn gravity_variation(self) -> f64 {
        match self {
            Self::Asteroid => 2.0 * Self::Asteroid.gravity_magnitude(),
            Self::Earth => 0.0698,
            Self::Mars => 0.0279,
            Self::Moon => 0.0253,
            Self::Orbit => 0.0,
        }
    }

    /// Range of gravitational acceleration in m/s² calculated as the magnitude ± variation/2.
    #[must_use] pub fn gravity_range(self) -> (f64, f64) {
        let magnitude = self.gravity_magnitude();
        let delta = self.gravity_variation() / 2.0;
        (magnitude - delta, magnitude + delta)
    }

    /// Intensity of Solar light in W/m².
    ///
    /// # Notes
    ///
    /// - Asteroid: Taken at 2.7 AU.
    /// - Earth | Mars: Taken at the surface. The peak value (sunny day) is subtracted by half of the variation.
    /// - Moon | Orbit: Taken at 1 AU.
    #[must_use] pub fn light_intensity(self) -> f64 {
        match self {
            Self::Asteroid => 190.0,
            Self::Earth => 1000.0 - Self::Earth.light_intensity_variation() / 2.0,
            Self::Mars => 842.0 - Self::Mars.light_intensity_variation() / 2.0,
            Self::Moon | Self::Orbit => 1361.0,
        }
    }

    /// Difference between the maximum and minimum of Solar light intensity in W/m².
    ///
    /// # Notes
    ///
    /// - Asteroid: Approximate range between 2.55 and 2.97 AU.
    /// - Earth | Mars: Guesstimated effect of atmosphere and weather.
    /// - Moon | Orbit: Minor variation due to elliptical orbit.
    #[must_use] pub fn light_intensity_variation(self) -> f64 {
        match self {
            Self::Asteroid => 50.0,
            Self::Earth => 450.0,
            Self::Mars => 226.0,
            Self::Moon | Self::Orbit => 0.5,
        }
    }

    /// Range of Solar light intensity in W/m² calculated as the intensity ± variation/2.
    #[must_use] pub fn light_intensity_range(self) -> (f64, f64) {
        let intensity = self.light_intensity();
        let delta = self.light_intensity_variation() / 2.0;
        (intensity - delta, intensity + delta)
    }

    /// Angular diameter of the Solar light source in degrees.
    ///
    /// # Assumptions
    ///
    /// - Earth | Mars: Taken at their distance from the Sun.
    /// - Asteroid | Moon | Orbit: Approximated as a point source due to lack of atmosphere.
    #[must_use] pub fn light_angular_diameter(self) -> f64 {
        match self {
            Self::Earth => 0.53,
            Self::Mars => 0.35,
            Self::Asteroid | Self::Moon | Self::Orbit => 0.0,
        }
    }

    /// Variation of the angular diameter of the Solar light source in degrees.
    #[must_use] pub fn light_angular_diameter_variation(self) -> f64 {
        match self {
            Self::Earth => 0.021,
            Self::Mars => 0.08,
            Self::Asteroid | Self::Moon | Self::Orbit => 0.0,
        }
    }

    /// Range of the angular diameter of the Solar light source in degrees calculated as the diameter ± variation/2.
    #[must_use] pub fn light_angular_diameter_range(self) -> (f64, f64) {
        let diameter = self.light_angular_diameter();
        let delta = self.light_angular_diameter_variation() / 2.0;
        (diameter - delta, diameter + delta)
    }

    /// Temperature of the Solar light source in K.
    ///
    /// # Assumptions
    ///
    /// - Earth | Mars: Guesstimated effect atmosphere and weather.
    /// - Asteroid | Moon | Orbit: Intrinsic color temperature of the Sun.
    #[must_use] pub fn light_color_temperature(self) -> f64 {
        match self {
            Self::Earth => 5750.0,
            Self::Mars => 6250.0,
            Self::Asteroid | Self::Moon | Self::Orbit => 5778.0,
        }
    }

    /// Variation of the temperature of the Solar light source in K.
    ///
    /// # Assumptions
    ///
    /// - Earth | Mars: Guesstimated effect atmosphere and weather.
    /// - Asteroid | Moon | Orbit: No significant variation.
    #[must_use] pub fn light_color_temperature_variation(self) -> f64 {
        match self {
            Self::Earth => 1500.0,
            Self::Mars => 500.0,
            Self::Asteroid | Self::Moon | Self::Orbit => 0.0,
        }
    }

    /// Range of the temperature of the Solar light source in K calculated as the temperature ± variation/2.
    #[must_use] pub fn light_color_temperature_range(self) -> (f64, f64) {
        let temperature = self.light_color_temperature();
        let delta = self.light_color_temperature_variation() / 2.0;
        (temperature - delta, temperature + delta)
    }
}

#[pymethods]
impl Scenario {
    #[getter("gravity_magnitude")]
    fn py_gravity_magnitude(&self) -> PyResult<f64> {
        Ok(self.gravity_magnitude())
    }

    #[getter("gravity_variation")]
    fn py_gravity_variation(&self) -> PyResult<f64> {
        Ok(self.gravity_variation())
    }

    #[getter("gravity_range")]
    fn py_gravity_range(&self) -> PyResult<(f64, f64)> {
        Ok(self.gravity_range())
    }

    #[getter("light_intensity")]
    fn py_light_intensity(&self) -> PyResult<f64> {
        Ok(self.light_intensity())
    }

    #[getter("light_intensity_variation")]
    fn py_light_intensity_variation(&self) -> PyResult<f64> {
        Ok(self.light_intensity_variation())
    }

    #[getter("light_intensity_range")]
    fn py_light_intensity_range(&self) -> PyResult<(f64, f64)> {
        Ok(self.light_intensity_range())
    }

    #[getter("light_angular_diameter")]
    fn py_light_angular_diameter(&self) -> PyResult<f64> {
        Ok(self.light_angular_diameter())
    }

    #[getter("light_angular_diameter_variation")]
    fn py_light_angular_diameter_variation(&self) -> PyResult<f64> {
        Ok(self.light_angular_diameter_variation())
    }

    #[getter("light_angular_diameter_range")]
    fn py_light_angular_diameter_range(&self) -> PyResult<(f64, f64)> {
        Ok(self.light_angular_diameter_range())
    }

    #[getter("light_color_temperature")]
    fn py_light_color_temperature(&self) -> PyResult<f64> {
        Ok(self.light_color_temperature())
    }

    #[getter("light_color_temperature_variation")]
    fn py_light_color_temperature_variation(&self) -> PyResult<f64> {
        Ok(self.light_color_temperature_variation())
    }

    #[getter("light_color_temperature_range")]
    fn py_light_color_temperature_range(&self) -> PyResult<(f64, f64)> {
        Ok(self.light_color_temperature_range())
    }
}
