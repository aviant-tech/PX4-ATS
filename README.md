# PX4 ATS (Automatic Trigger System)

A standalone PX4 module that monitors the main flight controller and triggers parachute deployment when a failure is detected.

The ATS is intended to run on a dedicated Cube Purple (`cubepurple_ats` build target).

Configure using the `AV_ATS*` parameters.

## SITL Tests

Build the ATS SITL target:

```bash
DONT_RUN=1 make px4_sitl_ats
```

Install dependencies and run:

```bash
cd test/ats_tests
pip install -r requirements.txt
python3 -m pytest test_ats.py -v
```
