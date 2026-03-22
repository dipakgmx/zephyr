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

* **Heart Rate Service (HRS)** -- measurement notifications and control point
  writes are fully protected by ACS.

Unprotected characteristics (GAP Device Name, DIS Manufacturer Name, etc.)
remain accessible via plain GATT.

The sample supports all ACS cipher algorithms (GCM, CCM, GMAC, CMAC) and key
exchange methods (ECDH, KDF, OOB) through composable Kconfig overlays.

Protection Model
================

ACS uses *restriction maps* to declare which GATT resources require which
security configuration.  This sample defines two maps:

* **Map 0** -- native-service map (``map_isc_id = NONE``).  DIS characteristics
  are listed here with ``ISC_ID_NONE``, accessible without key exchange.

* **Map 1** (``CONFIG_BT_ACS_ACTIVE_RMAP_ID``) -- protected map.  CTS and HRS
  characteristics require ``ISC_ID_DEFAULT`` for write/notify operations.
  The ACS CP ``Get Restriction Map Descriptor`` opcode is itself protected.

Requirements
************

* A board with Bluetooth LE support and PSA Crypto (e.g. nrf52840dk, frdm_rw612)
* A BLE central that implements the Authorization Control Profile (ACP), or
  the Bluetooth PTS tool for compliance testing

Building and Running
********************

Default build (GCM + all key exchange methods)
----------------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash

CCM with SEQ_DIFF_FIXED nonce + ECDH only
------------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-cipher-ccm-diff-fixed.conf;overlay-kex-ecdh.conf"

GMAC (integrity only, no confidentiality) + ECDH
-------------------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-cipher-gmac.conf;overlay-kex-ecdh.conf"

ECDH with P-384 curve
---------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-curve-p384.conf"

PTS compliance (full-blown + bonding)
-------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/peripheral_acs
   :board: frdm_rw612
   :goals: build flash
   :gen-args: -DEXTRA_CONF_FILE="overlay-bonding.conf"

Overlay Reference
=================

Cipher overlays (mutually exclusive):

* ``overlay-cipher-ccm-diff-fixed.conf`` -- AES-128-CCM, SEQ_DIFF_FIXED nonce
* ``overlay-cipher-ccm-evenodd.conf`` -- AES-128-CCM, SEQ_EVEN_ODD nonce
* ``overlay-cipher-cmac.conf`` -- AES-128-CMAC (authentication only)
* ``overlay-cipher-gmac.conf`` -- AES-128-GMAC (integrity + authentication)

CCM tag size (compose with a CCM cipher overlay):

* ``overlay-ccm-mac4.conf`` -- 4-byte authentication tag (minimum)
* ``overlay-ccm-mac16.conf`` -- 16-byte authentication tag (maximum)

Key exchange overlays (restrict to single method):

* ``overlay-kex-ecdh.conf`` -- ECDH only (P-256 default)
* ``overlay-kex-kdf.conf`` -- KDF only (HKDF-SHA-256 default)
* ``overlay-kex-oob.conf`` -- OOB only

Curve overlays (compose with ECDH):

* ``overlay-curve-p384.conf`` -- NIST P-384
* ``overlay-curve-p521.conf`` -- NIST P-521
* ``overlay-curve-25519.conf`` -- Curve25519

KDF hash overlays (compose with KDF):

* ``overlay-kdf-sha384.conf`` -- HKDF-SHA-384
* ``overlay-kdf-sha512.conf`` -- HKDF-SHA-512

Feature overlays:

* ``overlay-bonding.conf`` -- session persistence + multi-bond + initiate pairing
* ``overlay-security-switch.conf`` -- enable Set Security Controls Switch procedure
* ``overlay-settings-shell.conf`` -- shell for bond management

Minimal build overlay:

* ``overlay-minimal.conf`` -- single cipher (GCM) + single KEX (ECDH), no optional features
