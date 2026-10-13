macro_rules! python_module_name {
    () => {
        ::const_format::str_split!(::std::module_path!(), "::")
            .last()
            .unwrap()
    };
}

macro_rules! python_module_name_full {
    () => {
        ::const_format::str_replace!(
            ::const_format::str_replace!(::std::module_path!(), "::", "."),
            ::std::env!("CARGO_PKG_NAME"),
            $crate::PYTHON_MODULE_NAME
        )
    };
}

macro_rules! python_add_submodule {
    ($parent_module:ident, $module:ident) => {{
        let py = $parent_module.py();
        $parent_module.add_submodule(&$module)?;
        py.import_bound(::pyo3::intern!(py, "sys"))?
            .getattr(::pyo3::intern!(py, "modules"))?
            .set_item($crate::macros::python_module_name_full!(), $module)
    }};
}

pub(crate) use {python_add_submodule, python_module_name, python_module_name_full};
