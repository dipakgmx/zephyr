.. zephyr:code-sample:: ble_peripheral_acs
   :name: ACS Peripheral (CTS + HRS)
   :relevant-api: bt_acs bt_cts bt_hrs bluetooth

   Protect standard BLE services (CTS, HRS) behind ACS with encrypted
   data channels and ECDH key exchange.

Overview
********

This sample demonstrates the Authorization Control Service (ACS) protecting
two standard BLE services on a peripheral device:

* **Current Time Service (CTS)** -- mixed per-opcode protection: read is
  unprotected, write and notify require ACS encryption via Data In / Data Out.

* **Heart Rate Service (HRS)** -- measurement notifications, the Body Sensor
  Location read, and control point writes are protected by ACS.

Unprotected characteristics (GAP Device Name, DIS Manufacturer Name, etc.)
remain accessible via plain GATT.

Key exchange uses Curve P-256 with HKDF-SHA-256, as required by the
Authorization Control Profile.  Both the ECDH and standalone KDF methods are
always available.  The data-protection algorithm is selected through the
cipher overlays below.

Protection Model
================

ACS uses *restriction maps* to declare which GATT resources require which
Information Security Configuration (ISC).  Resources that are not listed in a
map take the map's default ISC, so only protected resources need declaring.

The library always registers two maps:

* **Map 0** -- reserved for direct, non-mediated access.
* **Map 1** -- the unprotected map, where every resource sits at ISC 0.

This sample adds its own protected map as
``CONFIG_BT_ACS_ACTIVE_RMAP_ID`` (2 by default), made active on every
connection.  Its entries are declared next to the services they protect, in
:file:`src/sample_cts.c` and :file:`src/sample_hrs.c`.

Requirements
************

* A board with Bluetooth LE support and PSA Crypto (e.g. nrf52840dk, frdm_rw612)
* A BLE central that implements the Authorization Control Profile (ACP), or
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

Overlay Reference
=================

Cipher overlays.  GCM is always compiled in on a confidentiality-capable
server (ACP 1.0 Section 4.1.1.2); GMAC and CMAC drop to a lower protection
level first.  Each restriction map operation entry names a single ISC, so the
overlay also decides which cipher governs the sample's protected resources:

* ``overlay-cipher-ccm.conf`` -- AES-128-CCM, added alongside GCM and used
  for the protected resources
* ``overlay-cipher-cmac.conf`` -- AES-128-CMAC (authentication only)
* ``overlay-cipher-gmac.conf`` -- AES-128-GMAC (integrity + authentication)

Feature overlays:

* ``overlay-confirm-oob.conf`` -- advertise every ECDH confirmation method
  and action; the base build offers Output Numeric only

See :file:`Testing.md` for the PTS execution order.
