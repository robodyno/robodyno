# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.6.1-fix]

### Added
- `LedDriver` component([!10](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/10)).
- `GpsSensor` component([!11](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/11)).
- `Battery` component([!13](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/13)).
- Robodyno Pro absolute motor models.

### Fixed
- Module `robodyno.robots` was missing the `ThreeDofCartesian` class([!9](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/9)).
- Component `robodyno.components.can_bus.motor` could not recognize the state of the motor when rebooting([!14](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/14)).
- CLI `robodyno monitor` occasionally could not be interrupted by `Ctrl+C`([!14](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/14)).

## [1.6.1] - 2023-05-29

### Added

- Documentations for Robodyno.
- Examples written in Jupyter Notebook.

### Changed

- License from MIT to Apache 2.0.
- Constructors of `CanBusDevice` now accept `can`, `id_` and `type_` as parameters instead of `iface`, `id` and `type`.
- Constructors of `WebotsDevice` now accept `webots`, `id_` and `type_` as parameters instead of `iface`, `id` and `type`.
- Enhance the `CanBus` interface to receive messages more efficiently.
- The `CanBus` interface can now be used to receive and send messages independently.
- Rewrite the `Webots` interface and corresponding classes.
- Enhance the CLI tool to support more commands and to be more user-friendly.

### Fixed

- Example of `motor.config_can_bus` in robodyno/README.md was incorrect ([!1](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/1)).'
- The motor direction was inconsistent between Webots simulation and actual operation ([!2](http://101.42.250.169:8081/robodyno/robodyno/-/merge_requests/2)).

## [1.6.0] - 2023-05-05

### Added

- Mkdocs configurations.
- Changelog file.
- Gitlab CI workflows.

### Changed

- Project structure.
