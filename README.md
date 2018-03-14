Autonomous Cleaning Robot (ACRo)

[![Build Status](https://travis-ci.com/udeyrishi/ACRo.svg?token=ez7psV6XvuSyQ3hU3b5M&branch=master)](https://travis-ci.com/udeyrishi/ACRo)


## Wiring

### L293D

* Blacks -> Ground
* Red -> Enable
* Greens -> Coil A
* Blues -> Coil B
* Brown -> 3.3V logic power in (straight from Pi)
* Orange -> 12V motor power in (from power circuit, capable of supplying 400 mA)
* Yellows -> 3.3V GPIO control outputs from Pi