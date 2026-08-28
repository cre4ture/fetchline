//! Pure STS packet handling and controller-session ownership helpers.
//!
//! This module deliberately has no hardware dependencies.  Firmware calls the
//! same functions after receiving UART bytes, while the native test package can
//! exercise the packet rules without requiring an ESP32-C3 test harness.

pub const STS_HEADER: [u8; 2] = [0xff, 0xff];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InstructionPacketError {
    OutputTooSmall,
    ParametersTooLong,
}

/// Encodes an STS instruction packet into `output` and returns its byte length.
pub fn encode_instruction_packet(
    output: &mut [u8],
    id: u8,
    instruction: u8,
    parameters: &[u8],
) -> Result<usize, InstructionPacketError> {
    let length = parameters
        .len()
        .checked_add(2)
        .and_then(|length| u8::try_from(length).ok())
        .ok_or(InstructionPacketError::ParametersTooLong)?;
    let packet_len = usize::from(length) + 4;
    if output.len() < packet_len {
        return Err(InstructionPacketError::OutputTooSmall);
    }

    output[..2].copy_from_slice(&STS_HEADER);
    output[2] = id;
    output[3] = length;
    output[4] = instruction;
    output[5..packet_len - 1].copy_from_slice(parameters);
    output[packet_len - 1] = checksum(&output[2..packet_len - 1]);
    Ok(packet_len)
}

/// Returns the STS inverted checksum for all bytes after the two-byte header.
pub const fn checksum(bytes: &[u8]) -> u8 {
    let mut sum = 0_u8;
    let mut index = 0;
    while index < bytes.len() {
        sum = sum.wrapping_add(bytes[index]);
        index += 1;
    }
    !sum
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StatusPacketError {
    InvalidLength,
    UnexpectedId,
    InvalidChecksum,
}

#[derive(Debug, Eq, PartialEq)]
pub struct StatusPacket<'a> {
    pub error: u8,
    pub parameters: &'a [u8],
}

/// Validates a complete STS status packet body (without the `FF FF` header).
///
/// The packet consists of `id`, `length`, `error`, zero or more parameters,
/// and a checksum.  Firmware bounds `length` at 66 because its STS response
/// buffer holds at most 64 parameters.
pub fn decode_status_packet(
    packet: &[u8],
    expected_id: u8,
) -> Result<StatusPacket<'_>, StatusPacketError> {
    if packet.len() < 4 {
        return Err(StatusPacketError::InvalidLength);
    }

    let id = packet[0];
    let length = usize::from(packet[1]);
    if !(2..=66).contains(&length) || packet.len() != length + 2 {
        return Err(StatusPacketError::InvalidLength);
    }
    if id != expected_id {
        return Err(StatusPacketError::UnexpectedId);
    }
    if checksum(&packet[..packet.len() - 1]) != packet[packet.len() - 1] {
        return Err(StatusPacketError::InvalidChecksum);
    }

    Ok(StatusPacket {
        error: packet[2],
        parameters: &packet[3..packet.len() - 1],
    })
}

/// A monotonically increasing ownership marker for controller sessions.
///
/// The MCU accepts two TCP handshakes at a time, but only the session holding
/// the current epoch may issue a command to the shared STS bus.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControllerEpoch(u64);

impl ControllerEpoch {
    pub const fn new() -> Self {
        Self(0)
    }

    pub fn claim(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(1);
        self.0
    }

    pub const fn is_current(self, epoch: u64) -> bool {
        self.0 == epoch
    }
}

/// Serializes controller ownership changes until the retiring TCP transport is
/// listening again.  It permits exactly one active controller generation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControllerAdmission {
    epoch: ControllerEpoch,
    active: bool,
    listener_required: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControllerAdmissionResult {
    Granted {
        generation: u64,
        replaces_active: bool,
    },
    WaitForListener,
}

impl ControllerAdmission {
    pub const fn new() -> Self {
        Self {
            epoch: ControllerEpoch::new(),
            active: false,
            listener_required: false,
        }
    }

    /// Grants the next session only when no predecessor is still returning its
    /// TCP transport to listening mode.
    pub fn try_claim(&mut self) -> ControllerAdmissionResult {
        if self.listener_required {
            return ControllerAdmissionResult::WaitForListener;
        }

        let replaces_active = self.active;
        let generation = self.epoch.claim();
        self.active = true;
        self.listener_required = replaces_active;
        ControllerAdmissionResult::Granted {
            generation,
            replaces_active,
        }
    }

    /// Marks the current session as no longer active after a normal disconnect.
    pub fn release(&mut self, generation: u64) {
        if self.epoch.is_current(generation) {
            self.active = false;
        }
    }

    /// Returns true when this listener transition unblocks a queued takeover.
    pub fn listener_ready(&mut self) -> bool {
        let required = self.listener_required;
        self.listener_required = false;
        required
    }

    pub const fn is_current(self, generation: u64) -> bool {
        self.epoch.is_current(generation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_a_ping_packet() {
        let mut packet = [0_u8; 16];
        let length = encode_instruction_packet(&mut packet, 5, 0x01, &[]).unwrap();
        assert_eq!(&packet[..length], &[0xff, 0xff, 5, 2, 1, 247]);
    }

    #[test]
    fn encodes_a_register_read_packet() {
        let mut packet = [0_u8; 16];
        let length = encode_instruction_packet(&mut packet, 5, 0x02, &[56, 2]).unwrap();
        assert_eq!(&packet[..length], &[0xff, 0xff, 5, 4, 2, 56, 2, 186]);
    }

    #[test]
    fn rejects_an_oversized_instruction_parameter_list() {
        let mut packet = [0_u8; 260];
        let parameters = [0_u8; 254];
        assert_eq!(
            encode_instruction_packet(&mut packet, 1, 3, &parameters),
            Err(InstructionPacketError::ParametersTooLong)
        );
    }

    #[test]
    fn rejects_a_too_small_instruction_output_buffer() {
        let mut packet = [0_u8; 7];
        assert_eq!(
            encode_instruction_packet(&mut packet, 1, 3, &[40, 1]),
            Err(InstructionPacketError::OutputTooSmall)
        );
    }

    #[test]
    fn decodes_a_status_packet_with_parameters() {
        let packet = [5, 4, 0, 0x34, 0x12, 0xb0];
        let status = decode_status_packet(&packet, 5).unwrap();
        assert_eq!(status.error, 0);
        assert_eq!(status.parameters, &[0x34, 0x12]);
    }

    #[test]
    fn retains_the_servo_reported_error() {
        let packet = [5, 2, 0x20, 0xd8];
        let status = decode_status_packet(&packet, 5).unwrap();
        assert_eq!(status.error, 0x20);
        assert!(status.parameters.is_empty());
    }

    #[test]
    fn rejects_malformed_status_packets() {
        assert_eq!(
            decode_status_packet(&[5, 4, 0, 0x34, 0x12, 0xb1], 5),
            Err(StatusPacketError::InvalidChecksum)
        );
        assert_eq!(
            decode_status_packet(&[6, 2, 0, 0xf7], 5),
            Err(StatusPacketError::UnexpectedId)
        );
        assert_eq!(
            decode_status_packet(&[5, 67, 0, 0], 5),
            Err(StatusPacketError::InvalidLength)
        );
    }

    #[test]
    fn newest_controller_epoch_wins() {
        let mut epoch = ControllerEpoch::new();
        let first = epoch.claim();
        assert!(epoch.is_current(first));
        let second = epoch.claim();
        assert!(!epoch.is_current(first));
        assert!(epoch.is_current(second));
    }

    #[test]
    fn serializes_takeovers_until_a_listener_returns() {
        let mut admission = ControllerAdmission::new();
        let ControllerAdmissionResult::Granted {
            generation: first,
            replaces_active,
        } = admission.try_claim()
        else {
            panic!("the first controller must be admitted");
        };
        assert!(!replaces_active);

        let ControllerAdmissionResult::Granted {
            generation: second,
            replaces_active,
        } = admission.try_claim()
        else {
            panic!("the second controller must replace the first");
        };
        assert!(replaces_active);
        assert!(!admission.is_current(first));
        assert!(admission.is_current(second));
        assert_eq!(
            admission.try_claim(),
            ControllerAdmissionResult::WaitForListener
        );

        assert!(admission.listener_ready());
        let ControllerAdmissionResult::Granted {
            generation: third,
            replaces_active,
        } = admission.try_claim()
        else {
            panic!("a listener return must admit the waiting controller");
        };
        assert!(replaces_active);
        assert!(admission.is_current(third));
    }

    #[test]
    fn normal_disconnect_does_not_block_the_next_controller() {
        let mut admission = ControllerAdmission::new();
        let ControllerAdmissionResult::Granted {
            generation: first, ..
        } = admission.try_claim()
        else {
            panic!("the first controller must be admitted");
        };
        admission.release(first);
        let ControllerAdmissionResult::Granted {
            replaces_active, ..
        } = admission.try_claim()
        else {
            panic!("a disconnected controller must not block a replacement");
        };
        assert!(!replaces_active);
    }
}
