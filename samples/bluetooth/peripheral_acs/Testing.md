Peripheral ACS Sample — PTS Compliance Guide

Sample location: zephyr/samples/bluetooth/peripheral_acs/
Supported boards: frdm_rw612, rd_rw612_bga, nrf52840dk

---
Overlay Model

prj.conf is the full-blown base: GCM cipher + ECDH + KDF + OOB key exchange.

  Cipher overlays  replace the data protection algorithm (override prj.conf defaults).
  kex overlays     restrict key exchange to a single method (disable the other two).
  overlay-bonding  adds session persistence and INITIATE_PAIRING opcode.

Overlays compose additively — apply one cipher overlay and/or one kex overlay.

---
Common to all configurations (run these first, every time)

  ACS/SR/SGGIT/SER/BV-01-C   Service GGIT – Authorization Control
  ACS/SR/SGGIT/CHA/BV-02-C   Characteristic GGIT – ACS Status
  ACS/SR/SGGIT/CHA/BV-03-C   Characteristic GGIT – ACS Data In
  ACS/SR/SGGIT/CHA/BV-04-C   Characteristic GGIT – ACS Data Out Notify
  ACS/SR/SGGIT/CHA/BV-05-C   Characteristic GGIT – ACS Data Out Indicate
  ACS/SR/SGGIT/CHA/BV-06-C   Characteristic GGIT – ACS Control Point
  ACS/SR/ACSCP/BV-01-C       Get All Active Descriptors without Key Descriptor
  ACS/SR/ACSCP/BV-14-C       Get Information Security Configuration Descriptor
  ACS/SR/ACSCP/BV-16-C       Get Key Descriptor
  ACS/SR/ACSCP/BV-29-C       Get ACS Feature
  ACS/SR/CR/BV-01-C          Read characteristic with protected resource
  ACS/SR/ACSCP/BV-21-C       Invalidate All Established Security
  ACS/SR/ACSCP/BV-22-C       Invalidate All Established Security for a Protected Resource
  ACS/SR/ACSCP/BV-25-C       Abort
  ACS/SR/ACSCP/BV-05-C       Get Restriction Map Descriptor
  ACS/SR/ACSCP/BV-06-C       Get Restriction Map Descriptor based on Resource Handle Filter

---
### Default: GCM + ECDH + KDF + OOB

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs

  PTS ICS:  GCM=y  ECDH=y  KDF=y  OOB=y  SEQ_DIFF_FIXED=y

  ACS/SR/ACSCP/BV-31-C   ECDH key exchange
  ACS/SR/ACSCP/BV-32-C   KDF key exchange
  ACS/SR/ACSCP/BV-30-C   OOB key exchange
  ACS/SR/ACSCP/BV-33-C   Set AC Client Nonce Fixed (4-byte fixed part)
  ACS/SR/CW/BV-01-C      Protected write — HRS Control Point (Data In)
  ACS/SR/ACSCP/BI-07-C   ECDH — invalid public key
  ACS/SR/ACSCP/BI-08-C   ECDH — invalid confirmation code
  ACS/SR/ACSCP/BI-13-C   ECDH — invalid AC Client confirmation random
  ACS/SR/ACSCP/BI-12-C   KDF — Procedure Not Applicable (run after disabling KDF in ICS)

---
### Bonding + Initiate Pairing (overlay-bonding.conf)

  Adds: session key persistence across reconnects, INITIATE_PAIRING opcode (0xDF).
  Required for any test that reconnects and expects an existing session.

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-bonding.conf

  PTS ICS:  GCM=y  ECDH=y  KDF=y  OOB=y  Initiate Pairing=y

  All tests from the default build, plus:
  ACS/SR/ACSCP/BV-35-C   Initiate Pairing

  Note: BV-35-C sequence is — complete key exchange → client writes opcode 0xDF →
        server calls bt_conn_set_security(L2) → SMP pairing completes.

---
### ECDH only (overlay-kex-ecdh.conf)

  Disables KDF and OOB. Simplest ICS setup for focused ECDH testing.

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-kex-ecdh.conf

  PTS ICS:  GCM=y  ECDH=y  KDF=n  OOB=n  SEQ_DIFF_FIXED=y

  ACS/SR/ACSCP/BV-31-C   ECDH key exchange
  ACS/SR/ACSCP/BV-33-C   Set AC Client Nonce Fixed
  ACS/SR/CW/BV-01-C      Protected write
  ACS/SR/ACSCP/BI-07-C   ECDH — invalid public key
  ACS/SR/ACSCP/BI-08-C   ECDH — invalid confirmation code
  ACS/SR/ACSCP/BI-13-C   ECDH — invalid AC Client confirmation random

---
### KDF only (overlay-kex-kdf.conf)

  Disables ECDH and OOB. Tests pre-shared key derivation path only.
  PSK must be provisioned on device before PTS initiates BV-32-C.

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-kex-kdf.conf

  PTS ICS:  GCM=y  ECDH=n  KDF=y  OOB=n  SEQ_DIFF_FIXED=y

  ACS/SR/ACSCP/BV-32-C   KDF key exchange
  ACS/SR/ACSCP/BV-33-C   Set AC Client Nonce Fixed
  ACS/SR/CW/BV-01-C      Protected write
  ACS/SR/ACSCP/BI-12-C   KDF — Procedure Not Applicable

---
### OOB only (overlay-kex-oob.conf)

  Disables ECDH and KDF. Tests static OOB numeric confirmation path.
  OOB PSK must be loaded into oob_psk[] before running BV-30-C.

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-kex-oob.conf

  PTS ICS:  GCM=y  ECDH=n  KDF=n  OOB=y  SEQ_DIFF_FIXED=y

  ACS/SR/ACSCP/BV-30-C   OOB key exchange
  ACS/SR/ACSCP/BV-33-C   Set AC Client Nonce Fixed
  ACS/SR/CW/BV-01-C      Protected write

---
### CCM SEQ_DIFF_FIXED (overlay-cipher-ccm-difff-fixed.conf)

  AES-128-CCM, 8-byte fixed nonce prefix + 5-byte counter, MAC=8 bytes.

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-cipher-ccm-difff-fixed.conf

  PTS ICS:  CCM=y  SEQ_DIFF_FIXED=y  SEQ_EVEN_ODD=n  ECDH=y  KDF=y  OOB=y

  ACS/SR/ACSCP/BV-31-C   ECDH key exchange
  ACS/SR/ACSCP/BV-33-C   Set AC Client Nonce Fixed (8-byte fixed part)
  ACS/SR/CW/BV-01-C      Protected write
  ACS/SR/ACSCP/BI-09-C   Set AC Client Nonce Fixed — Invalid Operand 1
  ACS/SR/ACSCP/BI-10-C   Set AC Client Nonce Fixed — Invalid Operand 2
  ACS/SR/ACSCP/BI-11-C   Set AC Client Nonce Fixed — Invalid Operand 3
  ACS/SR/GEH/BI-08-C     Invalid Nonce with Sequence Number Different Fixed Parts

---
### CCM SEQ_EVEN_ODD (overlay-cipher-ccm-evenodd.conf)

  AES-128-CCM, even nonces for server / odd nonces for client, no fixed part.
  SET_CLIENT_NONCE_FIXED not applicable — uncheck BV-33-C in PTS ICS.

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-cipher-ccm-evenodd.conf

  PTS ICS:  CCM=y  SEQ_EVEN_ODD=y  SEQ_DIFF_FIXED=n  BV-33-C=unchecked

  ACS/SR/ACSCP/BV-31-C   ECDH key exchange
  ACS/SR/CW/BV-01-C      Protected write
  ACS/SR/GEH/BI-07-C     Invalid Nonce with Sequence Number Even-Odd

---
### GMAC (overlay-cipher-gmac.conf)

  AES-128-GMAC: data authenticated but NOT encrypted (integrity only).
  Wire format: MAC(LSO, 16B) || Plaintext(LSO) — MAC precedes data.
  ISC ID: BT_ACS_ISC_ID_INTEGRITY (0x0004)

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-cipher-gmac.conf

  PTS ICS:  GMAC=y (INTEGRITY=y, CONFIDENTIALITY=n)  GCM=n  SEQ_DIFF_FIXED=y

  ACS/SR/ACSCP/BV-31-C   ECDH key exchange
  ACS/SR/ACSCP/BV-32-C   KDF key exchange
  ACS/SR/ACSCP/BV-30-C   OOB key exchange
  ACS/SR/ACSCP/BV-33-C   Set AC Client Nonce Fixed
  ACS/SR/CW/BV-01-C      Protected write (plaintext + MAC tag)
  ACS/SR/ACSCP/BI-07-C   ECDH — invalid public key
  ACS/SR/ACSCP/BI-08-C   ECDH — invalid confirmation code

---
### CMAC (overlay-cipher-cmac.conf)

  AES-128-CMAC: MAC-only, no nonce, no confidentiality, no integrity (ACS sense).
  SET_CLIENT_NONCE_FIXED not applicable — uncheck BV-33-C in PTS ICS.
  ISC ID: BT_ACS_ISC_ID_MAC_ONLY (0x0005)

  west build -b frdm_rw612 zephyr/samples/bluetooth/peripheral_acs \
    -- -DEXTRA_CONF_FILE=overlay-cipher-cmac.conf

  PTS ICS:  CMAC=y  INTEGRITY=n  CONFIDENTIALITY=n  BV-33-C=unchecked

  ACS/SR/ACSCP/BV-31-C   ECDH key exchange
  ACS/SR/ACSCP/BV-32-C   KDF key exchange
  ACS/SR/ACSCP/BV-30-C   OOB key exchange
  ACS/SR/CW/BV-01-C      Protected write (plaintext + MAC tag)
  ACS/SR/ACSCP/BI-07-C   ECDH — invalid public key
  ACS/SR/ACSCP/BI-08-C   ECDH — invalid confirmation code

---
Known Issues

1. BV-33-C from restored session
   Symptom: "key state is 6 (must be IDLE)" on reconnect with bonded peer.
   Cause: commit ada9ad0ddd5 tightened IDLE+COMPLETE → IDLE-only.
   Workaround: IDLE check disabled with #if 0 in acs_cp.c handle_set_client_nonce_fixed.
   Status: under investigation.

2. GMAC verify failed -149 (PSA_ERROR_INVALID_SIGNATURE)
   Cause: acs_data.c #else branch had Data||MAC order; spec §3.2 requires MAC||Data.
   Fix: GMAC/CMAC added to CCM/GCM branch in encode and decode paths.
   Status: fixed.

3. BT_ACS_ISC_ID_MAC_ONLY / BT_ACS_ISC_ID_INTEGRITY build errors
   Cause: defines existed only in internal acs_isc.h, not public acs.h.
   Fix: both added to include/zephyr/bluetooth/services/acs.h.
   Status: fixed.
