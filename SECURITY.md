# Security Policy

Anateus is a Vex Robotics library. Hence, securoty is not one of the main
concerns here. However, if you do find a vulnerability that concerns you
greatly, here is a few stuff you need to know.

## Supported Versions
The last minor version (according to semantic versioning) of Antaeus will being
supported with security updates. Like mentioned above, security is not a large
concern as antaeus only mainly runs in embedded systems.

## What is considered a Vulnerability
> A software vulnerability is a security flaw, weakness, or bug in an
> application’s code, design, or configuration that can be exploited by
> malicious actors to compromise the confidentiality, integrity, or availability
> of systems and data.
The most likely vulnerability antaeus might face is malicious dependencies. If
such a dependency is deemed to be malicious, it will immediately be taken down,
especially is it was designed to attack the programmer's system.
### These are not vulnerabilities:
- Threading related bugs (e.g. deadlocks)
- The library panicking or having any similar critical error
- Any similar bug in the library that does not affect your own computer

## Reporting a Vulnerability

Vulnerabilities in Antaeus can directly be reported in Github. More on that
[here](https://docs.github.com/en/code-security/security-advisories/guidance-on-reporting-and-writing-information-about-vulnerabilities/privately-reporting-a-security-vulnerability).
If you cannot do so, you can directly email me at saturnyx@disroot.org.
