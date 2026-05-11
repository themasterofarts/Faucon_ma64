cmake_minimum_required(VERSION 3.8)
project(faucon_drone)

find_package(ament_cmake REQUIRED)

# ── Python executables ────────────────────────────────────────────────────────

# Velocity controller (sim fallback — no PX4 required)
install(PROGRAMS
  scripts/drone_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drone_velocity_controller
)

# PX4 X500 adapter — named without .py so symlink-install creates a live symlink
install(PROGRAMS
  adapters/px4_x500_adapter
  DESTINATION lib/${PROJECT_NAME}
)

# ── Resource directories ──────────────────────────────────────────────────────

install(DIRECTORY adapters/    DESTINATION share/${PROJECT_NAME}/adapters)
install(DIRECTORY description/ DESTINATION share/${PROJECT_NAME}/description)
install(DIRECTORY config/      DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/      DESTINATION share/${PROJECT_NAME}/launch)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
