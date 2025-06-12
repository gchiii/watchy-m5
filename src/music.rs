use core::marker::PhantomData;

use thiserror_no_std::Error;
use allocator_api2::vec::Vec;
use defmt::error;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::{Duration, Timer};
use esp_hal::{ledc::{channel::{self, ChannelIFace}, timer::{self, TimerIFace}, HighSpeed, Ledc}, time::Rate};
use esp_hal::ledc::channel::Error as LedcChannelError;
use esp_hal::ledc::timer::Error as LedcTimerError;

#[allow(unused)]
// Note frequencies in Hertz as f64
pub const NOTE_B0: f64 = 31.0;
pub const NOTE_C1: f64 = 33.0;
pub const NOTE_CS1: f64 = 35.0;
pub const NOTE_D1: f64 = 37.0;
pub const NOTE_DS1: f64 = 39.0;
pub const NOTE_E1: f64 = 41.0;
pub const NOTE_F1: f64 = 44.0;
pub const NOTE_FS1: f64 = 46.0;
pub const NOTE_G1: f64 = 49.0;
pub const NOTE_GS1: f64 = 52.0;
pub const NOTE_A1: f64 = 55.0;
pub const NOTE_AS1: f64 = 58.0;
pub const NOTE_B1: f64 = 62.0;
pub const NOTE_C2: f64 = 65.0;
pub const NOTE_CS2: f64 = 69.0;
pub const NOTE_D2: f64 = 73.0;
pub const NOTE_DS2: f64 = 78.0;
pub const NOTE_E2: f64 = 82.0;
pub const NOTE_F2: f64 = 87.0;
pub const NOTE_FS2: f64 = 93.0;
pub const NOTE_G2: f64 = 98.0;
pub const NOTE_GS2: f64 = 104.0;
pub const NOTE_A2: f64 = 110.0;
pub const NOTE_AS2: f64 = 117.0;
pub const NOTE_B2: f64 = 123.0;
pub const NOTE_C3: f64 = 131.0;
pub const NOTE_CS3: f64 = 139.0;
pub const NOTE_D3: f64 = 147.0;
pub const NOTE_DS3: f64 = 156.0;
pub const NOTE_E3: f64 = 165.0;
pub const NOTE_F3: f64 = 175.0;
pub const NOTE_FS3: f64 = 185.0;
pub const NOTE_G3: f64 = 196.0;
pub const NOTE_GS3: f64 = 208.0;
pub const NOTE_A3: f64 = 220.0;
pub const NOTE_AS3: f64 = 233.0;
pub const NOTE_B3: f64 = 247.0;
pub const NOTE_C4: f64 = 262.0;
pub const NOTE_CS4: f64 = 277.0;
pub const NOTE_D4: f64 = 294.0;
pub const NOTE_DS4: f64 = 311.0;
pub const NOTE_E4: f64 = 330.0;
pub const NOTE_F4: f64 = 349.0;
pub const NOTE_FS4: f64 = 370.0;
pub const NOTE_G4: f64 = 392.0;
pub const NOTE_GS4: f64 = 415.0;
pub const NOTE_A4: f64 = 440.0;
pub const NOTE_AS4: f64 = 466.0;
pub const NOTE_B4: f64 = 494.0;
pub const NOTE_C5: f64 = 523.0;
pub const NOTE_CS5: f64 = 554.0;
pub const NOTE_D5: f64 = 587.0;
pub const NOTE_DS5: f64 = 622.0;
pub const NOTE_E5: f64 = 659.0;
pub const NOTE_F5: f64 = 698.0;
pub const NOTE_FS5: f64 = 740.0;
pub const NOTE_G5: f64 = 784.0;
pub const NOTE_GS5: f64 = 831.0;
pub const NOTE_A5: f64 = 880.0;
pub const NOTE_AS5: f64 = 932.0;
pub const NOTE_B5: f64 = 988.0;
pub const NOTE_C6: f64 = 1047.0;
pub const NOTE_CS6: f64 = 1109.0;
pub const NOTE_D6: f64 = 1175.0;
pub const NOTE_DS6: f64 = 1245.0;
pub const NOTE_E6: f64 = 1319.0;
pub const NOTE_F6: f64 = 1397.0;
pub const NOTE_FS6: f64 = 1480.0;
pub const NOTE_G6: f64 = 1568.0;
pub const NOTE_GS6: f64 = 1661.0;
pub const NOTE_A6: f64 = 1760.0;
pub const NOTE_AS6: f64 = 1865.0;
pub const NOTE_B6: f64 = 1976.0;
pub const NOTE_C7: f64 = 2093.0;
pub const NOTE_CS7: f64 = 2217.0;
pub const NOTE_D7: f64 = 2349.0;
pub const NOTE_DS7: f64 = 2489.0;
pub const NOTE_E7: f64 = 2637.0;
pub const NOTE_F7: f64 = 2794.0;
pub const NOTE_FS7: f64 = 2960.0;
pub const NOTE_G7: f64 = 3136.0;
pub const NOTE_GS7: f64 = 3322.0;
pub const NOTE_A7: f64 = 3520.0;
pub const NOTE_AS7: f64 = 3729.0;
pub const NOTE_B7: f64 = 3951.0;
pub const NOTE_C8: f64 = 4186.0;
pub const NOTE_CS8: f64 = 4435.0;
pub const NOTE_D8: f64 = 4699.0;
pub const NOTE_DS8: f64 = 4978.0;
pub const REST: f64 = 0.0; // No sound, for pauses


/// NoteLength is from music notation like quarter notes, eigth notes, whole notes 
/// the absolute value of the number is the divisor. ie 1 is a whole note, 2 is for a half note etc
/// the sign indicates whether or not the note is dotted (which means it is slightly longer)
#[derive(Clone, Copy)]
pub struct NoteLength (pub i16);

impl NoteLength {
    pub fn is_dotted(&self) -> bool {
        let NoteLength(l) = self;
        *l < 0
    }
    pub fn duration_ms(&self, whole_note: u32) -> u32 {
        let NoteLength(divider) = *self;
        if self.is_dotted() {
            let duration = whole_note / divider.unsigned_abs() as u32;
            (duration as f64 * 1.5) as u32
        } else {
            whole_note / divider as u32
        }
    }    
}


#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NoteData (pub f64, pub i16);

impl NoteData {
    pub fn to_note(&self, whole_note: u32) -> Note {
        let Self(freq_hz, divider) = self;
        let duration_ms = { 
            if *divider > 0 {
                whole_note / *divider as u32
            } else {
                let duration = whole_note / divider.unsigned_abs() as u32;
                (duration as f64 * 1.5) as u32
            }
        } as u64;
        let frequency = Rate::from_hz(*freq_hz as u32);
        Note { frequency, duration_ms }
    }
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Note {
    pub frequency: Rate, 
    pub duration_ms: u64,
}

#[derive(Debug, Error)]
pub enum BuzzerError {
    TimerError(#[from] LedcTimerError),
    ChannelError(#[from] LedcChannelError),
    NoSong,
}

#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum BuzzerState<'a> {
    #[default] 
    Stopped,
    Paused(Song<'a>),
    Playing(Song<'a>),
}

impl BuzzerState<'_> {
    async fn execute(self, rx: Receiver<'static, CriticalSectionRawMutex, BuzzerCommand<'static>, 3>) -> Self {
        let state = self;
        match (state, rx.receive().await) {
            (BuzzerState::Stopped, BuzzerCommand::Play(song)) => BuzzerState::Playing(song),
            (BuzzerState::Paused(song), BuzzerCommand::Resume) => BuzzerState::Playing(song),
            (BuzzerState::Paused(_), BuzzerCommand::Play(song)) => BuzzerState::Playing(song),
            (BuzzerState::Playing(song), BuzzerCommand::Pause) => BuzzerState::Paused(song),
            (BuzzerState::Playing(_), BuzzerCommand::Play(song)) => BuzzerState::Playing(song),
            (BuzzerState::Stopped, BuzzerCommand::Pause) => BuzzerState::Stopped,
            (BuzzerState::Stopped, BuzzerCommand::Resume) => BuzzerState::Stopped,
            (BuzzerState::Paused(song), BuzzerCommand::Pause) => BuzzerState::Paused(song),
            (BuzzerState::Playing(song), BuzzerCommand::Resume) => BuzzerState::Playing(song),
        }
    }
}

#[derive(Clone, PartialEq)]
pub enum BuzzerCommand<'a> {
    Pause,
    Resume,
    Play(Song<'a>),
}


pub struct Buzzer<'a> {
    ledc: Ledc<'static>, 
    pin: esp_hal::peripherals::GPIO2<'static>,
    rx: Receiver<'static, CriticalSectionRawMutex, BuzzerCommand<'static>, 3>,
    pub state: BuzzerState<'a>,
}


impl<'a> Buzzer<'a> {
    pub fn new(ledc: Ledc<'static>, pin: esp_hal::peripherals::GPIO2<'static> , rx: Receiver<'static, CriticalSectionRawMutex, BuzzerCommand, 3>) -> Self {
        Self { ledc, pin, rx, state: BuzzerState::default() }
    }
    async fn alternate_play_note(&self, note: Note) -> Result<(), BuzzerError>{
        let mut hstim = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
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

        Ok(())
    }

    pub async fn execute(&self, mut state: BuzzerState<'a>) -> BuzzerState<'a>{
        let rx = self.rx;
        state = match state {
            BuzzerState::Playing(mut song) => {
                match song.next_note() {
                    Some(note) => {
                        if let Err(_e) = self.alternate_play_note(note).await {
                            error!("problem playing note");
                            BuzzerState::Stopped
                        } else {
                            BuzzerState::Playing(song)
                        }
                    },
                    None => BuzzerState::Stopped,
                }
            },
            BuzzerState::Stopped => BuzzerState::Stopped,
            BuzzerState::Paused(song) => BuzzerState::Paused(song),
        };
        state.execute(rx).await
    }

}


#[derive(Debug, Clone, PartialEq)]
pub struct Song<'s> {
    whole_note: u32,
    pub notes: Vec<NoteData>,
    index: usize,
    _phantom: PhantomData<&'s NoteData>,
}


impl Song<'_> {
    pub fn new(tempo: u16, some_notes: &[NoteData]) -> Self {
        let whole_note = (60_000 * 4) / tempo as u32;
        let notes: Vec<NoteData> = some_notes.into();
        Self { whole_note, notes, _phantom: PhantomData, index: 0 }
    }
    
    fn create_note(&self, note: &NoteData) -> Note {
        note.to_note(self.whole_note)
    }

    pub fn next_note(&mut self) -> Option<Note> {
        let index = self.index;
        if let Some(n) = self.notes.get(index) {
            self.index = index + 1;
            Some(self.create_note(n))
        } else {
            None
        }
    }

    pub fn calc_note_duration(&self, divider: i16) -> u32 {
        if divider > 0 {
            self.whole_note / divider as u32
        } else {
            let duration = self.whole_note / divider.unsigned_abs() as u32;
            (duration as f64 * 1.5) as u32
        }
    }
}

