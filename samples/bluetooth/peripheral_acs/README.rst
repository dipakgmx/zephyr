.. zephyr:code-sample:: ble_peripheral_acs
   :name: ACS Peripheral (CTS + HRS)
   :relevant-api: bt_acs bt_cts bt_hrs bluetooth

   Protect standard BLE services (CTS, HRS) behind ACS with encrypted
   data channels and ECDH key exchange.

Overview
********

This sample demonstrates the Authorization Control Service (ACS) protecting
two standard BLE services on a peripheral device:

* **Current Time Service (CTS)** -- read, write and notify require ACS
  encryption via Data In / Data Out.

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

The library registers no map of its own.  This sample defines two in
:file:`src/main.c`:

* ``locked_map`` (ID 1) -- protects every CTS and HRS resource the sample
  declares.  It is mapped to ISC 0 in the Restriction Map ID List, so its
  descriptor is read on the plain ACS CP.  It is the map active at start,
  selected with ``BT_ACS_INITIAL_RESTRICTION_MAP()``.
* ``relaxed_map`` (ID 2) -- protects only reads and writes of Current Time, and
  Invalidate All Established Security and Set Security Controls Switch.  It is
  mapped to the sample's ISC, so requests about the map itself go over the ACS
  Data path.

A map's own ISC protects only requests about that map.  The resources and ACS
CP procedures each map protects:

.. list-table::
   :header-rows: 1

   * - Resource or procedure
     - ``locked_map``
     - ``relaxed_map``
   * - CTS Current Time read and write
     - Data In, Data Out
     - Data In, Data Out
   * - CTS Current Time notification
     - Data Out
     - plain ATT
   * - HRS Heart Rate Measurement notification, Body Sensor Location read,
       Heart Rate Control Point write
     - Data In, Data Out
     - plain ATT
   * - Get Restriction Map Descriptor, Activate Restriction Map (for the map
       named in the request)
     - ACS CP
     - Data In, answered on Data Out Indicate
   * - Get All Active Descriptors (while the map is active)
     - ACS CP
     - Data In, answered on Data Out Indicate
   * - Invalidate All Established Security, Set Security Controls Switch
       (while the map is active)
     - ACS CP
     - Data In, answered on Data Out Indicate
   * - Get Restriction Map ID List, Get ISC Descriptor, Get Key Descriptor, key
       exchange, Invalidate Key, Abort
     - ACS CP
     - ACS CP

Every procedure a client without keys needs stays on the plain ACS CP under
either map, so it can always learn which ISC it needs and establish security.
With two maps registered, the Feature Response reports Multiple Restriction
Maps Supported and Activate Restriction Map switches between them.  The active
map is not stored, so the locked map is active again after every reset.

The protected resources are declared next to the services they protect, in
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
