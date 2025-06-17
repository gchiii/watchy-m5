use crate::music::Note;

use {esp_backtrace as _, esp_println as _};


use thiserror_no_std::Error;
use defmt::{error, info, Format};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, DynamicReceiver}};
use embassy_time::{Duration, Timer};
use esp_hal::ledc::{channel::{self, ChannelIFace}, timer::{self, TimerIFace}, HighSpeed, Ledc};
use esp_hal::ledc::channel::Error as LedcChannelError;
use esp_hal::ledc::timer::Error as LedcTimerError;

pub type BuzzerChannel = Channel::<CriticalSectionRawMutex, BuzzerCommand, 3>;
pub type BuzzerSender<'a> = embassy_sync::channel::DynamicSender<'a, BuzzerCommand>;
pub type BuzzerReceiver<'a> = embassy_sync::channel::DynamicReceiver<'a, BuzzerCommand>;

#[derive(Debug, Error)]
pub enum BuzzerError {
    TimerError(#[from] LedcTimerError),
    ChannelError(#[from] LedcChannelError),
    NoSong,
}

#[derive(Debug, Clone, PartialEq)]
pub enum BuzCommand {
    Stop,
    Play(Note),
}


#[derive(Debug, Clone, PartialEq)]
pub enum BuzzerCommand {
    Stop,
    Play(Note),
}

impl Format for BuzzerCommand {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            BuzzerCommand::Stop => defmt::write!(fmt, "Stop"),
            BuzzerCommand::Play(_note) => defmt::write!(fmt, "Play"),
        }
    }
}

#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum BuzzerState {
    #[default] 
    Stopped,
    Playing(Note),
}

impl BuzzerState {
    // async fn execute(self, rx: Receiver<'static, CriticalSectionRawMutex, BuzzerCommand<'static>, 3>) -> Self {
    async fn execute(self, rx: DynamicReceiver<'_, BuzzerCommand>) -> Self {
        let state = self;
        let cmd = rx.receive().await;
        info!("recieved cmd {}", cmd);
        match (state, cmd) {
            (BuzzerState::Stopped, BuzzerCommand::Stop) => BuzzerState::Stopped,
            (BuzzerState::Stopped, BuzzerCommand::Play(note)) => BuzzerState::Playing(note),
            (BuzzerState::Playing(_note), BuzzerCommand::Stop) => BuzzerState::Stopped,
            (BuzzerState::Playing(_note), BuzzerCommand::Play(new_note)) => BuzzerState::Playing(new_note),
        }
     }
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
    async fn play_note(&self, note: Note) -> Result<BuzzerState, BuzzerError>{
        if note.frequency.as_hz() > 0 {
            let mut hstim = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
            info!("frequency = {}", note.frequency);
            hstim
                .configure(timer::config::Config {
                    duty: timer::config::Duty::Duty10Bit,
                    clock_source: timer::HSClockSource::APBClk,
                    frequency: note.frequency,
                })?;
    
            let mut channel0 = self.ledc.channel(channel::Number::Channel0, unsafe { self.pin.clone_unchecked() });
            channel0
                .configure(channel::config::Config {
                    timer: &hstim,
                    duty_pct: 50,
                    pin_config: channel::config::PinConfig::PushPull,
                })?;
            let pause_duration = note.duration_ms / 10; // 10% of duration_ms
    
            Timer::after(Duration::from_millis(note.duration_ms - pause_duration)).await;
            channel0.set_duty(0)?;
            Timer::after(Duration::from_millis(pause_duration)).await;
        }

        Ok(BuzzerState::Stopped)
    }

    pub async fn execute(&self, mut state: BuzzerState) -> BuzzerState{
        let rx = self.rx;
        state = match state {
            BuzzerState::Stopped => BuzzerState::Stopped,
            BuzzerState::Playing(note) => {
                if let Err(_e) = self.play_note(note).await {
                    error!("error playing note!");
                }
                BuzzerState::Stopped
            },
        };
        state.execute(rx).await
    }

}
