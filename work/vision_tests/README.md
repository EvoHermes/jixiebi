# Regression scope
The harness copies the current A130 vision controller and USART3 parser. Hardware, interrupt masking and IK results are stubbed. Run `run_tests.cmd` with Visual Studio 2022 Community.

`verify_pi_v002.py <release-folder>` loads the Pi v002 coordinate encoder and checks its packets, edges, directions, invalid frames, and split HOME response against the firmware parser.

This is a host snapshot. Regenerate copied sections after firmware changes. It does not validate physical motion.
