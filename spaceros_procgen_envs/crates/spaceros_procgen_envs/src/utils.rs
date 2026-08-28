pub const SUPPORTED_FILE_EXTENSIONS: &[&str] = &[
    #[cfg(feature = "json")]
    "json",
    #[cfg(feature = "toml")]
    "toml",
    #[cfg(feature = "yaml")]
    "yaml",
    #[cfg(feature = "yaml")]
    "yml",
];
