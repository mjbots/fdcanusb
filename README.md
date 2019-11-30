# fdcanusb #

The fdcanusb provides a USB interface to an FDCAN bus.  It presents a serial-like interface over the USB port.

# Protocol #

Each command or response is a single newline terminated line, composed
of 7-bit ASCII encoded characters.  Every command sent from the client
to the device has a response which consists of an optional command
specific response followed by a final line consisting solely of `OK` or
`ERR <details>`.

## States ##

The device can be in one of three states.

1. *Bus Off*: This is the initial state of the device.  In this state,
no frames may be sent or received, but the device may be configured.

2. *Bus On*: In this state frames may be sent and received, but
configuration is locked out.

3. *Error*: If enabled, the device may enter an error state upon
certain bus conditions.  If in this state, then an error clear command
must be sent to re-enter the "Bus Off" state.

# Commands #

Sample commands are prefixed with `>` to show the data sent from the
client to the device and `<` to show the values sent from the device
to the client.  Those characters are for illustrative purposes only,
and are not present in actual communications.

## *conf* ##

The "conf" command is used to read and set configuration values.  It
has several subcommands.

### *conf enumerate* ###

This emits all current configuration values one after another.

### *conf get* ###

This queries the value of one specific configuration item.

```
>conf get sample.value
<31235
<OK
```

### *conf set* ###

This sets the value of one specific configuration itme.

```
>conf set sample.value 1234
<OK
```

### *conf load* ###

This loads all persistent configuration from the persistent storage.

```
>conf load
<OK
```

### *conf write* ###

This stores all configuration values to persistent storage.  They will
then be the default when the device is powered on the in the future.

```
>conf write
<OK
```

### *conf default* ###

Reset all configuration values to their "default" state.

```
>conf default
<OK
```

## can ##

### *can on* ###

Enter the "Bus On" state.  All configurable values are validated and
put into place.

### *can off* ###

Enter the "Bus Off" state.  Transmission and reception of CAN messages
is halted, and configurable values may be changed.

### *can clear* ###

Clear the "Error" state.  This will re-enter the "Bus Off" state.

### *can std* ###

Send a standard CAN frame.  The following format is used.

`can std <HEXID> <HEXDATA> <options>`

`options` may be zero or more of the following optionally separated by
spaces.

1. *B/b* require/disable bitrate switching
2. *F/f* require/disable FDCAN format
3. *R/r* require/disable remote frame


### *can ext* ###

Send an extended CAN frame.  The following format is used.

`can ext <HEXID> <HEXDATA> <options>`

The allowable options are the same as for `can std`.

### *can status* ###

Report the current mode and status of various flags.

```
>can status
<mode:BUSON
<OK
```

# Receiving CAN Frames #

When in the "Bus On" state, the device may spontaneously emit the
following lines upon receipt of valid CAN frames or other errors.

## *rcv std* ##

A standard CAN frame has been received.

```
<rcv <HEXID> <HEXDATA> <flags>
```

`flags` may be zero of more of the following separated by spaces:

1. *E/e* frame was received with extended/classic ID
1. *B/b* frame was received with/without bitrate switching
2. *F/f* frame was received in fdcan/classic mode
3. *R/r* frame was remote/data frame
4. tNNNNN timestamp of receipt measured in microseconds
5. fNN integer ID of which filter matched this frame

# Configurable Values #

The following items may be configured.

* *can.bitrate* - The bitrate used for the CAN frame (or header only
  if bitrate switching is used.
* *can.fd_bitrate* - The bitrate used for the data and CRC field when
  bitrate switching is used.
* *can.automatic_retransmission* - When true/non-zero, frames will be
  retried until acknowledged.
* *can.fdcan_frame* - By default, send frames in FDCAN mode
* *can.bitrate_switch* - By default, switch to the `can.fd_bitrate`
  for the data and CRC fields.
* *can.restricted_mode* - Only transmit acknowledgements.
* *can.bus_monitor* - Transmit nothing.

* *can.filter.N.id1* - first ID
* *can.filter.N.id2* - second ID or bitmask
* *can.filter.N.mode*
  * 0 - range
  * 1 - exact match
  * 2 - bitmask
* *can.filter.N.type*
  * 0 - standard
  * 1 - extended
* *can.filter.N.action*
  * 0 - accept
  * 1 - reject
