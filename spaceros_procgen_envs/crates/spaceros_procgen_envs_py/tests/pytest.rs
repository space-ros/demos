//! Run all python tests with pytest.

#[test]
fn pytest() {
    // Arrange
    let python_exe = std::env::var("PYO3_PYTHON")
        .or_else(|_| std::env::var("ISAAC_SIM_PYTHON"))
        .unwrap_or("python3".to_string());
    let mut command = std::process::Command::new(python_exe);
    let command = command.arg("-m").arg("pytest");

    // Act
    let output = command.output().unwrap();
    println!("{}", std::str::from_utf8(&output.stdout).unwrap());
    eprintln!("{}", std::str::from_utf8(&output.stderr).unwrap());

    // Assert
    assert!(output.status.success());
}
