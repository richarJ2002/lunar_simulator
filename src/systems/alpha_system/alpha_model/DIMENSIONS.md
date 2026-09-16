# Alpha ExoMars geometry basis

All dimensions are in metres unless noted otherwise. The model uses `+X`
forward, `+Y` left, and `+Z` up.

## Published dimensions used directly

| Quantity | Model value | Published value | Source |
|---|---:|---:|---|
| Wheel-centre track | 1.200 | 1200 mm | DLR EXM-BB2 report, Table 3.1 |
| Front-to-centre axle spacing | 0.640 | 640 mm | DLR EXM-BB2 report, Table 3.1 |
| Centre-to-rear axle spacing | 0.720 | 720 mm | DLR EXM-BB2 report, Table 3.1 |
| Front-to-rear wheelbase | 1.360 | 640 + 720 mm | Derived from the two published spacings |
| Flight wheel diameter | 0.285 | 285 mm | Poulakis et al., Table 1 |
| Flight wheel width | 0.120 | 120 mm | Poulakis et al., Table 1 |
| DEP/STR/DRV actuator envelope | diameter 0.125, length 0.1262 | diameter 125 mm x 126.2 mm | Grandy et al., Section 4 |

The DLR report states that EXM-BB2 uses the same triple-bogie suspension
design as ExoMars at 1:1 scale. Its own older test wheels are 250 x 110 mm;
the Alpha model deliberately uses the later published flight-BEMA wheel
envelope of 285 x 120 mm instead.

## Bogie layout in the model

| Bogie | Passive pivot | Wheel endpoints | Verified endpoint span |
|---|---|---|---:|
| Front-left | `(0.320, +0.600, 0.540)` | front-left `(0.640, +0.600)`, centre-left `(0, +0.600)` | 0.640 |
| Front-right | `(0.320, -0.600, 0.540)` | front-right `(0.640, -0.600)`, centre-right `(0, -0.600)` | 0.640 |
| Rear | `(-0.720, 0, 0.540)` | rear-left `(-0.720, +0.600)`, rear-right `(-0.720, -0.600)` | 1.200 |

## Reference-derived visual dimensions

The flight-production drawings do not publicly specify the bogie beam plate
profile, beam depth, wall thickness, pivot housing, or interface clearances.
Those dimensions must not be presented as confirmed flight dimensions. The
following visual envelopes were scaled from Figures 2.4, 3.1, and 3.2 of the
DLR report and Figures 2-4 of the BEMA papers:

| Visual quantity | Model value | Status |
|---|---:|---|
| Side shaped-beam length | 0.720 | Endpoint span plus 40 mm visual overhang per end |
| Side shaped-beam maximum depth | 0.130 | Scaled visual approximation |
| Side shaped-beam thickness | 0.090 | Scaled visual approximation |
| Rear shaped-beam length | 1.280 | Endpoint span plus 40 mm visual overhang per end |
| Rear shaped-beam maximum depth | 0.130 | Scaled visual approximation |
| Rear shaped-beam thickness | 0.090 | Scaled visual approximation |
| Passive-pivot housing diameter | 0.210 | Scaled visual approximation |

The body collision envelope is 1.250 x 1.050 x 0.500 and is offset 40 mm
forward. This leaves at least 30 mm between the body and the shaped side-beam
envelopes and at least 70 mm between the body and rear pivot housing. Thus the
rear bogie is behind, rather than inside, the body.

## References

1. DLR, *Wheel Walking for Improving the Rover Mobility on Soft Soils and
   Slopes*, DLR-IB-RM-OP-2017-201, sections 2.1.2 and 3.1, especially Table
   3.1 and Figures 2.4, 3.1, and 3.2:
   https://elib.dlr.de/114850/1/DLR-IB-RM-OP-2017-201.pdf
2. P. Poulakis et al., *Overview and Development Status of the ExoMars Rover
   Mobility Subsystem*, ASTRA 2015, sections 3.1-3.3 and Table 1:
   https://www.researchgate.net/publication/290445739_Overview_and_Development_Status_of_the_ExoMars_Rover_Mobility_Subsystem
3. D. Grandy et al., *Development and Qualification of the ExoMars Bogie
   Electro-Mechanical Assembly (BEMA) Rotary Actuators*, ESMATS 2019,
   sections 3.2 and 4:
   https://esmats.eu/esmatspapers/pastpapers/pdfs/2019/grandy.pdf
