(longplay) (digital) audio delay/looper (proof of concept)
=============================================

even if there are solid commercial solutions for the
plausible use cases, the aim here is to create a
working low-budget basis that can be customized for
special needs. (also, the cheaper commercial solutions
seem to have a delay/loop time or channel count that is
not satisfactory)

also consider this a proof of concept for using stm32
for simple audio effect units..

basic architecture
-----------------------------

An interrupt handler at sampling frequency performs the following operations:

1. get new sample from ADC
2. pass new sample to storage logic
3. get old sample from storage logic (using given delay)
4. pass a half-wet mix to DAC

The storage logic stores or retrieves single blocks (in polling mode) once the
buffers as used by above interrupt handler are full/empty.
Alternative/possible operation/optimization:
- Use bigger buffers to allow for writes/reads of more than one block at a time -> singular delays exceeding the timing restrictions may thus be spread over several blocks.
- Use non-blocking sdio operation.
- Combine sdio read/write operation instead of separating logic -> if delay is align blockwise block buffers are full/empty at the same time

data throughput
-----------------------------

for sampling rate of 48khz and 8bit sample resolution
needs throughput of at least 48kHz x 1 bytes = 48kByte / sec
/ channel; or given storage blocks of 512 bytes: 48k x
1 / 512 = 93.75 combined average block read+writes / sec, or a
max time limit of 10.666.. ms per read/write operation.

furthermore 48kB / sec / channel means a required
storage space of 48kB * 60 = 2.81... MB / minute /
channel or ~164.79... MB / hour / channel.

microcontrollers have quite the memory restrictions thus making (longer)
storage of audio data impossible. SD cards offer themselves as
a cheap and accessible way to provide the memory capacity needed
(and could even be used as recorders).

the basic SPI interface of SD cards requires 7-10ms on
average; as tested on a nucleo-
f303k8.

spi test code: https://gist.github.com/tschiemer/c77ccae718c401fa7604f56b68034944

the 4bit SDIO mode (on nucleo-l476rg) seems to typically
require roughly 3200 usec for one combined read/write operation
occassionally significantly more (10+ ms) as tested on a stm32l476rg
with *suboptimal* code.

*Note*: a recorder or player only requires write or read operations (obviously) and
might have better sdio timing conditions

For basic throughput limitations given due to chosen testboards:
stm32l476rg:
"Data transfer up to 50 MHz for the 8 bit mode" (-> 25 MByte/s for 4 bit mode)
stm32h476zi2: "Data transfer up to 208 Mbyte/s for the 8-bit mode." (104 MByte/s for 4 bit mode)


AD/DA
--------------------------------------
(some) stm32s provide 12bit ADCs and DACs, possibly
opamps, so along with the SDIO interface they offer
themselves for a budget friendly basis.
nucleo-l476rg was chosen for the moment being.

Basic line in circuit that works for a mixer line level out signal:
```
                3V3
                 |
                 R1 100k
                 |
LineIn -----C1---x---- ADC
           0.1u  |
                 R2 100k
                 |
                GND
```
It is really suboptimal, especially if the 3V3 source is taken directly from the
nucleo board and other components (like the sdio) are connected to it aswell: noise
on the power line is clearly heard, also any noise from the power source (power adapter
or computer) is passed on.
But hey, it's a proof of concept.

Interesting: STM32 ADCs allow for differential input.

Also see: https://forum.arduino.cc/index.php?topic=567581.0

ADC optimziations as explained by stm:
https://www.st.com/resource/en/application_note/cd00004444-understanding-and-minimising-adc-conversion-errors-stmicroelectronics.pdf

Differential biasing techniques: http://ww1.microchip.com/downloads/en/appnotes/00842a.pdf

A line out circuit isn't necessarily needed (no guarantees are given for any attached equipment) but are suggested. Essentially the DC bias should be removed with an inline capacitor, a post-capacitor resistor connecting line to ground has been suggested to avoid backflow/charging of capacitor (as I understand). Audio equipment often comes with an input DC block already - but relying on this might risk your equipment.

STM32s often have two (buffered) DACs if they have any, so outputting a differential signal would be possible aswell.

For more: https://forum.allaboutcircuits.com/threads/removing-dc-offset-from-a-signal.65288/

https://www.allaboutcircuits.com/technical-articles/dac-output-circuitry-for-an-arbitrary-waveform-generator/


pin assignment
----------------
NUCLEO-L476RG

connector | pin | function
--- | --- | ---
CN7 | 1 / PC10 | D2 (sdio)
 | 2 / PC11 | D3 (sdio)
 | 3 / PC12 | CLK (sdio)
 | 4 / PD2 | CMD (sdio)
CN10 | 1 / PC9 | D1 (sdio)
 | 2 / PC8 | D2 (sdio)
CN8 | 1 / PA0 | ADC
 | 3 / PA4 | DAC


SD card:
 - https://www.sparkfun.com/products/12941
 - https://support.keith-koep.com/service/doku.php/service/hardware/appnote/connectsdcard

AD/DA: see above

further references
----------------

- https://github.com/stevstrong/Audio-sample
- https://github.com/frisnit/STM32L476-radar
- https://github.com/akospasztor/stm32-bootloader/blob/master/projects/STM32L496-CustomHw/include/bsp_driver_sd.h
- http://ugweb.cs.ualberta.ca/~c274/resources/hardware/SDcards/SD_SDIO_specsv1.pdf
