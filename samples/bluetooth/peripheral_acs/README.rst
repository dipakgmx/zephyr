.. zephyr:code-sample:: ble_peripheral_acs
   :name: ACS Peripheral (CTS + HRS)
   :relevant-api: bt_acs bt_cts bt_hrs bluetooth

   Protect standard Bluetooth LE services (CTS, HRS) behind ACS with protected
   data channels and ECDH key exchange.

Overview
********

This sample demonstrates the Authorization Control Service (ACS) protecting
two standard Bluetooth LE services on a peripheral device:

* **Current Time Service (CTS)** -- Current Time read, write and notify go
  over ACS Data In / Data Out.

* **Heart Rate Service (HRS)** -- measurement notifications, the Body Sensor
  Location read, and control point writes are protected by ACS.

Unprotected characteristics (GAP Device Name, CTS Local Time Information, DIS
Manufacturer Name, etc.) remain accessible via plain GATT.

Key exchange uses Curve P-256 with HKDF-SHA-256, as required by the
Authorization Control Profile.  Both the ECDH and standalone KDF methods are
always available.  The data-protection algorithm is selected through the
cipher overlays below.

Protection Model
================

ACS uses *restriction maps* to declare which GATT resources require which
Information Security Configuration (ISC).  Resources that are not listed in a
map take the map's default ISC, so only protected resources need declaring.

The library registers no map of its own.  This sample defines one in
:file:`src/main.c`, ``locked_map`` (ID 1), active at start through
``BT_ACS_INITIAL_RESTRICTION_MAP()``.  It protects:

* CTS Current Time read, write and notify
* HRS Heart Rate Measurement notify, Body Sensor Location read and Heart Rate
  Control Point write
* the ACS CP procedures Invalidate All Established Security and Set Security
  Controls Switch, which act on every client

A protected read or write goes over ACS Data In and is answered on ACS Data Out;
a protected notification goes on ACS Data Out.  The map's own ISC is 0, so a
client without keys reads the map descriptor on the plain ACS CP.  Every other
ACS CP procedure stays on the plain ACS CP, so a client without keys can always
learn which ISC it needs and establish security.

The protected resources are declared next to the services they protect, in
:file:`src/sample_cts.c` and :file:`src/sample_hrs.c`.

The sample sends a heart rate every 2 s and the current time every 10 s while a
client has ACS security, as reported by the ``security_established`` and
``security_invalidated`` callbacks.

PTS Layout
==========

``overlay-pts.conf`` sets ``CONFIG_SAMPLE_ACS_PTS``, which adds ``relaxed_map``
(ID 2), the layout the ACS PTS test cases expect:

* ``locked_map`` leaves Invalidate All Established Security and Set Security
  Controls Switch on the plain ACS CP.
* ``relaxed_map`` protects those two procedures and Current Time read and
  write.  Its map ISC is the sample's ISC, so Get Restriction Map Descriptor
  and Activate Restriction Map for it, and Get All Active Descriptors while it
  is active, go over ACS Data In.

With two maps the Feature Response reports Multiple Restriction Maps Supported
and Activate Restriction Map switches between them.  The active map is not
stored, so ``locked_map`` is active again after every reset.  The overlay also
fixes the Output OOB Number at 1.  Under ``locked_map`` any client can turn the
security controls off, so use the overlay for testing only.

Requirements
************

* A board with Bluetooth LE support and PSA Crypto (e.g. nrf52840dk, frdm_rw612)
* A Bluetooth LE central that implements the Authorization Control Profile (ACP), or
  the Bluetooth PTS tool for compliance testing

Building and Running
********************

Default build (AES-128-GCM)
---------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash

AES-128-GCM and AES-128-CCM
---------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-cipher-ccm.conf"

AES-128-GMAC (integrity only, no confidentiality)
-------------------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-cipher-gmac.conf"

AES-128-CMAC (authentication only)
----------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-cipher-cmac.conf"

Overlay Reference
=================

Cipher overlays.  GCM is always compiled in on a confidentiality-capable
server (ACP 1.0 Section 4.1.1.2); GMAC and CMAC drop to a lower protection
level first.  The sample names one ISC for all its protected resources, so the
overlay also decides which cipher protects them:

* ``overlay-cipher-ccm.conf`` -- AES-128-CCM, added alongside GCM and used
  for the protected resources
* ``overlay-cipher-cmac.conf`` -- AES-128-CMAC (authentication only)
* ``overlay-cipher-gmac.conf`` -- AES-128-GMAC (integrity + authentication)

Feature overlays:

* ``overlay-confirm-oob.conf`` -- advertise every ECDH confirmation method
  and action; the base build offers Output Numeric only
* ``overlay-pts.conf`` -- the restriction map layout for the ACS PTS test
  cases (see `PTS Layout`_); combine it with the other overlays, for example
  ``-DEXTRA_CONF_FILE="overlay-cipher-ccm.conf;overlay-pts.conf"``

Python AC Client
================

:file:`acs_client.py` is a minimal AC Client for a host with a Bluetooth LE
adapter.  It runs the ECDH key exchange (confirmation method None) and the KDF
key exchange, writes CTS Current Time and reads it back, reads HRS Body Sensor
Location over ACS Data In, and invalidates its keys.  Pass the option that
matches the build: ``--gcm`` (default build), ``--ccm``, ``--gmac`` or
``--cmac``.

.. code-block:: console

   pip install bleak cryptography
   python3 acs_client.py --gcm
