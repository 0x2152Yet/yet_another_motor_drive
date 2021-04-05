## Yet Another Motor Drive
Field-oriented control motor drive software for 3-phase permanent magnet
synchronous motors that supports both sensored and sensor-less operation.

### Overview
This project contains the software that implements the field-oriented control 
(FOC) of a 3-phase permanent magnet synchronous motor (PMSM). Support is 
included for controlling the motor either using a physical shaft encoder 
("sensors") or by estimating the motor's shaft angle using other sensed data 
("sensor-less").  

At present, this software is configured to operate on an STMicro STM32F439 processor.
The supported motor is a Segway I Series PT motor (these can be found on eBay).
The gate drive and power electronics are provide by a Texas Instruments DRV8312EVM
evaluation module (note that the DRV8312 is utilized without a processor installed).
The motor bus is operated nominally at 24.5 volts.  A combination of a power supply
and batteries are used to provide the motor bus power.

Please see the block diagram and setup notes in the documentation folder for more
information about the setup's configuration.

### Why Yet Another Motor Drive?
There are many fine motor drive software examples available. Many of the available 
examples, however, target very specific low-cost lower capability processors.
As a result, the software must often be coded in such a way to fit within the
constraints of the target processor.  These software examples are often marvels
of efficiency but they can often be rather difficult to follow.

This motor drive project is targeted to a processor that is rather more capable 
than the processors often used to control PMSM motors.  The software can take
advantage of this extra capability to concentrate on the PMSM FOC algorithms 
themselves without having to add the extra complexity required to fit within a
smaller processor's footprint.  As a result, the motor control portion of this
software:
  * Is not highly optimized for speed or size
  * Performs most of its calculations in floating point
  * Uses engineering units for most values.  For example, bus voltage is expressed
    in volts with 1.0 volt being represented by the value 1.0
  * Where possible, processor specific items are reserved for the low level
    hardware interfaces and are not present in the motor control algorithms
    themselves
    
### Status and Future Updates
At present, this software controls a single target motor using either a motor 
shaft angle sensed with an analog encoder or an estimated shaft angle computed 
using one of two available algorithms.  Up to this time all testing and loop tuning 
has performed with the motor operating without a load.  Data logging is
supported with a basic serial console and a high speed DAC output.

Planned updates include:
  * Verifying and improving performance with the motor under load
  * Adding support for a more commonly available motor (for example the Anaheim
    Automation motor included with the TI DRV8312EVM kit)
  * Improving the external data logging and motor control tools
  * More fully documenting the setup and its operation
    