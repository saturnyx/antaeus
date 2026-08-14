<br><br>
<div align="center">
    <img src="readme/logo-white-cropped.png" alt="Antaeus Logo" width="15%" style="margin-bottom: -50px;">
</div>
<h1 align="center" style="margin-top: -50px;">Ἀνταῖος</h1>
<p align="center">
    Antaeus is a versatile, next-gen framework based on <a href="https://vexide.dev">Vexide</a>
</p>

<p align="center">
    <a href="https://crates.io/crates/antaeus"><img alt="Crates.io Version" src="https://img.shields.io/crates/v/antaeus?style=flat-square&logo=rust&logoColor=white&color=CE422B&labelColor=24292F"></a>
    &nbsp;
    <a href="https://docs.rs/antaeus"><img alt="Docs.rs" src="https://img.shields.io/docsrs/antaeus?style=flat-square&logo=docs&logoColor=white&color=2563EB&labelColor=24292F"></a>
    &nbsp;
    <a href="./LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-7C3AED?style=flat-square&logo=opensourceinitiative&logoColor=white&labelColor=24292F"></a>
    &nbsp;
    <img alt="MSRV" src="https://img.shields.io/badge/MSRV-1.70-0F766E?style=flat-square&logo=rust&logoColor=white&labelColor=24292F">
</p>

# Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Philosophy](#philosophy)
  - [Extensibility](#extensibility)
  - [Single-Threading and Update-Based Architecture](#single-threading-and-update-based-architecture)
  - [70% Virtual Testing](#70-virtual-testing)
  - [Robustness and Performance over Simplicity](#robustness-and-performance-over-simplicity)
- [Comparison with Evian](#comparison-with-evian)
- [Using Antaeus](#using-antaeus)
  - [A Notice](#a-notice)
  - [Is Antaeus for you?](#is-antaeus-for-you)
  - [Quickstart](#quickstart)
  - [Adding to your project](#adding-to-your-project)
- [Development](#development)
  - [Current State of Development](#current-state-of-development)
- [License and Attribution](#license-and-attribution)
  - [License](#license)
  - [Use of AI](#use-of-ai)

# Introduction

Antaeus is one of the few frameworks and libraries based on Vexide. Built as an alternative to Evian, many of the algorithms used in this library are heavily modified to suit VEX robots.

# Features

- Algorithms:
  - Pure Pursuit (Custom Implementation)
  - Wheel-Based Localization (Odometry)
  - PID
  - Kalman Filter
- Display:
  - Embedded Graphics
- Misc:
  - Angle and Length Units
  - Logging
  - Geometry

# Philosophy

## Extensibility

Antaeus is built to be extensible. Users can add their own algorithms and integrate custom hardware easily. This philosophy was borrowed from Evian to ensure that the framework can evolve with the needs of its users.

## Single-Threading and Update-Based Architecture

One of the most unique features of Antaeus is that all algorithms run on a single main loop and are updated through a `tick` function. This saves memory and prevents race conditions. It also allows for easy integration of new algorithms and hardware without the need for complex threading or synchronization.

## 70% Virtual Testing

Antaeus was designed from the ground up to support full integration tests and unit tests. Rather than relying on physical testing, which can be time-consuming and error-prone, Antaeus allows for 70% of testing to be done virtually. Most algorithms in Antaeus can be tested in a virtual environment, which saves time and resources. This is especially useful for teams that may not have access to a physical robot at all times.

## Robustness and Performance over Simplicity

Many Vex libraries go by the "plug-and-play" or "easy-to-use" philosophy. While this is great for beginners, this library aims to be more advanced and robust. It is designed for users who are willing to put in the time to understand the underlying concepts and algorithms. Antaeus was designed to be used by people who are experts in Vex Robotics and want to push the limits of what is possible with their robots. It is not a library for beginners or those who are looking for a quick solution.

# Comparison with Evian

[Evian](https://github.com/vexide/evian) is another framework based on Vexide that inspired Antaeus. It aims to be an extensible framework that is easy to use. However, Antaeus aims to be a more advanced, modern framework that can bring VEX to a new level. It includes heavily modified algorithms suited specifically for VEX robots. It is bleeding-edge as well, which means that the API will often change between versions. Because of that, it is important to carefully pick which library you would like to use before continuing.

# Using Antaeus

## A Notice

It is important that you understand the concepts used in this library, such as odometry and pure pursuit. This library aims to be simple to implement, but it expects you to understand these concepts before using it and to learn more in the process. Please do not just "plug in" this library and use it without thinking much about it. If you have new ideas, you are strongly encouraged to contribute to this library and/or create your own library. More information on contributing can be found in [CONTRIBUTING.md](./CONTRIBUTING.md).

## Is Antaeus for you?

Antaeus is recommended for users who:
- need an advanced, cutting-edge library for autonomous programming,
- are willing to fix bugs by themselves when needed, and
- are willing to create their own algorithms (if you do, feel free to open a PR).

However, Antaeus is not recommended for users who:
- want an easy-to-use or plug-and-play library for their team,
- are a new team looking for a beginner-friendly library,
- do not know what pure pursuit, odometry, or a Kalman filter are, or
- have only a week left before competition and need to code fast.

## Quickstart

```rust
use std::{num::NonZeroU32, time::Duration};

use antaeus::{
    motion::feedback_control::pid::drive_pid::DrivePID,
    peripherals::drivetrain::differential::StandardDifferential,
    utils::units::Length,
};
use vexide::prelude::*;

// This is the main function
#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Let's first create our 4-motor drivetrain
    let drivetrain = StandardDifferential::new(
        [
            Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
            Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
        ],
        [
            Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
            Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
        ],
    );

    // Initializing PID for the drivetrain
    let mut pid = DrivePID::new(
        drivetrain,
        0.5,
        0.0,
        0.0,
        1000.0,
        Length::from_inches(3.25),
        NonZeroU32::new(4).unwrap(),
        NonZeroU32::new(4).unwrap(),
        Length::from_inches(13.0),
        Length::from_inches(0.0),
        Length::from_inches(0.5),
    );

    pid.set_relative_target(Length::from_inches(24.0), Length::from_inches(24.0)); // Moves forward
    pid.autotick(Duration::from_secs(5)).await;
}
```

More examples can be found in the [examples](./examples) folder.

## Adding to your project

Antaeus has been published to [crates.io](https://crates.io/crates/antaeus). To add Antaeus to your project, run the following command in your project directory:

```bash
cargo add antaeus
```

This will add the latest version of Antaeus to your `Cargo.toml` file. You can also specify a specific version by running:

```bash
cargo add antaeus@<version>
```

# Development

## Current State of Development

Antaeus is still developing. Many core features have been implemented, but we are still working on more advanced algorithms. We are also looking for contributors to help with the development process. If you are interested in contributing, please check out our [CONTRIBUTING.md](./CONTRIBUTING.md) file.

# License and Attribution

## License

Antaeus is licensed under the MIT License. This license permits you to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the software. For more information, please see the [LICENSE](./LICENSE) file.

## Use of AI

It is often impossible to completely negate the use of AI in our modern world. However, I have greatly limited the use of AI in this repo. Currently, more than 80% of source code is written by humans only.
