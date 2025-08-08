//! Buzzer driver and command interface for Watchy-M5.
//!
//! This module provides types and an async driver for controlling a piezo buzzer
//! using the ESP32 LEDC peripheral. It supports playing notes, rests, and stopping
//! the buzzer, with async command handling via Embassy channels.

use crate::music::{BuzzerNote, Note};

use {esp_backtrace as _, esp_println as _};

use thiserror_no_std::Error;
use defmt::{error, trace};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        HighSpeed, Ledc,
    },
    time::Rate,
};
use esp_hal::ledc::channel::Error as LedcChannelError;
use esp_hal::ledc::timer::Error as LedcTimerError;

/// Embassy channel for sending buzzer commands.
pub type BuzzerChannel = Channel::<CriticalSectionRawMutex, BuzzerCommand, 3>;
/// Dynamic sender for buzzer commands.
pub type BuzzerSender<'a> = embassy_sync::channel::DynamicSender<'a, BuzzerCommand>;
/// Dynamic receiver for buzzer commands.
pub type BuzzerReceiver<'a> = embassy_sync::channel::DynamicReceiver<'a, BuzzerCommand>;

/// Errors that can occur when controlling the buzzer.
#[derive(Debug, Error, defmt::Format)]
pub enum BuzzerError {
    /// Error from the LEDC timer peripheral.
    TimerError(#[from] LedcTimerError),
    /// Error from the LEDC channel peripheral.
    ChannelError(#[from] LedcChannelError),
}

/// Commands for controlling the buzzer.
#[derive(Debug, Clone, PartialEq)]
pub enum BuzzerCommand {
    /// Stop the buzzer (silence).
    Stop,
    /// Play a note or rest.
    Sound(BuzzerNote),
}

/// State of the buzzer.
#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum BuzzerState {
    /// The buzzer is stopped (silent).
    #[default]
    Stopped,
    /// The buzzer is playing a note.
    Playing(Note),
}

/// Async driver for the Watchy-M5 piezo buzzer.
///
/// The `Buzzer` struct manages the ESP32 LEDC peripheral and a GPIO pin to
/// generate tones. It receives commands via an async channel and updates its
/// state accordingly.
pub struct Buzzer<'a> {
    /// The LEDC peripheral for PWM generation.
    ledc: Ledc<'static>,
    /// The GPIO pin connected to the buzzer.
    pin: esp_hal::peripherals::GPIO2<'static>,
    /// The receiver for buzzer commands.
    rx: BuzzerReceiver<'a>,
    /// The current state of the buzzer.
    pub state: BuzzerState,
}

impl<'a> Buzzer<'a> {
    /// Creates a new `Buzzer` instance.
    ///
    /// # Arguments
    /// * `ledc` - The LEDC peripheral for PWM.
    /// * `pin` - The GPIO pin connected to the buzzer.
    /// * `rx` - The receiver for buzzer commands.
    ///
    /// # Returns
    /// A new `Buzzer` instance.
    pub fn new(ledc: Ledc<'static>, pin: esp_hal::peripherals::GPIO2<'static>, rx: BuzzerReceiver<'a>) -> Self {
        Self { ledc, pin, rx, state: BuzzerState::default() }
    }

    /// Waits for and executes the next buzzer command.
    ///
    /// This method receives a command from the channel and executes it,
    /// updating the buzzer state.
    ///
    /// # Arguments
    /// * `_state` - The current state (unused).
    ///
    /// # Returns
    /// The new state after executing the command.
    pub async fn execute(&mut self, mut _state: BuzzerState) -> BuzzerState {
        let rx = self.rx;
        let cmd = rx.receive().await;
        self.exec_cmd(cmd).await;
        BuzzerState::Stopped
    }

    /// Executes a single buzzer command asynchronously.
    ///
    /// Handles both `Stop` and `Sound` commands, updating the state and logging errors.
    ///
    /// # Arguments
    /// * `cmd` - The buzzer command to execute.
    pub async fn exec_cmd(&mut self, cmd: BuzzerCommand) {
        match cmd {
            BuzzerCommand::Stop => {
                match self.silence().await {
                    Ok(s) => {
                        self.state = s;
                    },
                    Err(_e) => {
                        error!("BuzzerError: {}", _e);
                        self.state = BuzzerState::Stopped;
                    },
                }
            },
            BuzzerCommand::Sound(buzzer_note) => {
                match self.buzz(buzzer_note).await {
                    Ok(s) => {
                        self.state = s;
                    },
                    Err(_e) => {
                        error!("BuzzerError: {}", _e);
                        self.state = BuzzerState::Stopped;
                    },
                }
            },
        }
    }

    /// Plays a note or rest asynchronously.
    ///
    /// For a rest, silences the buzzer for the specified duration.
    /// For a sound, plays the note at the given frequency and duration.
    ///
    /// # Arguments
    /// * `note` - The `BuzzerNote` to play.
    ///
    /// # Returns
    /// The resulting `BuzzerState` or an error.
    async fn buzz(&self, note: BuzzerNote) -> Result<BuzzerState, BuzzerError> {
        match note {
            BuzzerNote::Rest(dur) => {
                let now = Instant::now();
                let expires_at = now + Duration::from_millis_floor(dur as u64);
                let r = self.silence().await;
                Timer::at(expires_at).await;
                r
            },
            BuzzerNote::Sound(rate, dur) => {
                let now = Instant::now();
                let expires_at = now + Duration::from_millis_floor(dur as u64);
                let r = self.sound(rate, dur as u64).await;
                Timer::at(expires_at).await;
                r
            },
        }
    }

    /// Silences the buzzer by setting the duty cycle to zero.
    ///
    /// # Returns
    /// The resulting `BuzzerState` or an error.
    async fn silence(&self) -> Result<BuzzerState, BuzzerError> {
        let mut hstim = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
        hstim
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty10Bit,
                clock_source: timer::HSClockSource::APBClk,
                frequency: Rate::from_hz(440),
            })?;

        let mut channel0 = self.ledc.channel(channel::Number::Channel0, unsafe { self.pin.clone_unchecked() });
        channel0
            .configure(channel::config::Config {
                timer: &hstim,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })?;
        Timer::after(Duration::from_millis(2)).await;
        Ok(BuzzerState::Stopped)
    }

    /// Plays a sound at the specified frequency and duration.
    ///
    /// The buzzer is driven at 50% duty cycle for most of the duration, then silenced for a short pause.
    ///
    /// # Arguments
    /// * `freq` - The frequency to play.
    /// * `dur` - The duration in milliseconds.
    ///
    /// # Returns
    /// The resulting `BuzzerState` or an error.
    async fn sound(&self, freq: Rate, dur: u64) -> Result<BuzzerState, BuzzerError> {
        let pause_duration = dur / 10; // 10% of duration_ms
        let mut hstim = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
        trace!("frequency = {}", freq);
        hstim
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty10Bit,
                clock_source: timer::HSClockSource::APBClk,
                frequency: freq,
            })?;

        let mut channel0 = self.ledc.channel(channel::Number::Channel0, unsafe { self.pin.clone_unchecked() });
        channel0
            .configure(channel::config::Config {
                timer: &hstim,
                duty_pct: 50,
                pin_config: channel::config::PinConfig::PushPull,
            })?;

        Timer::after(Duration::from_millis(dur - pause_duration)).await;
        channel0.set_duty(0)?;
        Timer::after(Duration::from_millis(pause_duration)).await;
        Ok(BuzzerState::Stopped)
    }
}
