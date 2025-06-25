use crate::music::{BuzzerNote, Note};

use {esp_backtrace as _, esp_println as _};


use thiserror_no_std::Error;
use defmt::{error, trace};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use esp_hal::{ledc::{channel::{self, ChannelIFace}, timer::{self, TimerIFace}, HighSpeed, Ledc}, time::Rate};
use esp_hal::ledc::channel::Error as LedcChannelError;
use esp_hal::ledc::timer::Error as LedcTimerError;

pub type BuzzerChannel = Channel::<CriticalSectionRawMutex, BuzzerCommand, 3>;
pub type BuzzerSender<'a> = embassy_sync::channel::DynamicSender<'a, BuzzerCommand>;
pub type BuzzerReceiver<'a> = embassy_sync::channel::DynamicReceiver<'a, BuzzerCommand>;

#[derive(Debug, Error, defmt::Format)]
pub enum BuzzerError {
    TimerError(#[from] LedcTimerError),
    ChannelError(#[from] LedcChannelError),
}

#[derive(Debug, Clone, PartialEq)]
pub enum BuzzerCommand {
    Stop,
    Sound(BuzzerNote),
}



#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum BuzzerState {
    #[default] 
    Stopped,    
    Playing(Note),
}



pub struct Buzzer<'a> {
    ledc: Ledc<'static>, 
    pin: esp_hal::peripherals::GPIO2<'static>,
    rx: BuzzerReceiver<'a>,
    pub state: BuzzerState,
}


impl<'a> Buzzer<'a> {
    pub fn new(ledc: Ledc<'static>, pin: esp_hal::peripherals::GPIO2<'static> , rx: BuzzerReceiver<'a>) -> Self {
        Self { ledc, pin, rx, state: BuzzerState::default() }
    }

    pub async fn execute(&mut self, mut _state: BuzzerState) -> BuzzerState {
        let rx = self.rx;
        let cmd = rx.receive().await;
        self.exec_cmd(cmd).await;
        BuzzerState::Stopped
    }

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

    async fn buzz(&self, note: BuzzerNote) -> Result<BuzzerState, BuzzerError> {
        match note {
            BuzzerNote::Rest(_) => self.silence().await,
            BuzzerNote::Sound(rate, dur) => {
                self.sound(rate, dur as u64).await
            },
        }
    }

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
        Timer::after(Duration::from_millis(10)).await;
        Ok(BuzzerState::Stopped)
    }
    async fn sound(&self, freq: Rate, dur: u64) -> Result<BuzzerState, BuzzerError> {
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
            let pause_duration = dur / 10; // 10% of duration_ms
    
            Timer::after(Duration::from_millis(dur - pause_duration)).await;
            channel0.set_duty(0)?;
            Timer::after(Duration::from_millis(pause_duration)).await;
            Ok(BuzzerState::Stopped)
    }

}
