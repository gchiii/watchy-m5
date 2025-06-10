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


#[derive(Clone, Copy)]
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

#[derive(Debug, defmt::Format, Default)]
pub enum BuzzerState {
    #[default] 
    Silent,
    Play(Note),
}

impl BuzzerState {
    // pub async fn execute(self, buzzer: Buzzer) -> Result<self> {
    //     match self {
    //         BuzzerState::Silent => todo!(),
    //         BuzzerState::Play(tone) => buzzer,
    //     }
    // }
}


#[derive(Debug, Error)]
pub enum BuzzerError {
    TimerError(#[from] LedcTimerError),
    ChannelError(#[from] LedcChannelError),
}

pub struct Buzzer<'a> {
    ledc: Ledc<'static>, 
    buzzer: esp_hal::peripherals::GPIO2<'static>,
    rx: Receiver<'static, CriticalSectionRawMutex, BuzzerCommand, 3>,
    hstimer: Option<timer::Timer<'a, HighSpeed>>,
    channel: Option<channel::Channel<'a, HighSpeed>>,
    pub state: BuzzerState,
}


impl<'a> Buzzer<'a> {
    pub fn new(ledc: Ledc<'static>, buzzer: esp_hal::peripherals::GPIO2<'static>, rx: Receiver<'static, CriticalSectionRawMutex, BuzzerCommand, 3>) -> Self {
        Self { ledc, buzzer, rx, state: BuzzerState::Silent, hstimer: None, channel: None }
    }

    // pub async fn execute(&mut self) -> Result<BuzzerState> {
    //     match self.state {
    //         BuzzerState::Silent => {
    //             match self.rx.receive().await {
    //                 BuzzerCommand::Pause => todo!(),
    //                 BuzzerCommand::Resume => todo!(),
    //                 BuzzerCommand::Play(song) => todo!(),
    //             }
    //         },
    //         BuzzerState::Play(tone) => todo!(),
    //     }
    // }


    fn cfg_timer(&mut self, tone: Note) -> Result<timer::Timer<'a, HighSpeed>, BuzzerError> {
        let mut hstimer0: timer::Timer<'a, HighSpeed> = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
        hstimer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty10Bit,
                clock_source: timer::HSClockSource::APBClk,
                frequency: tone.frequency,
            })?;
        Ok(hstimer0)
    }

    fn cfg_channel(&'a mut self, tone: Note) -> Result< &'a mut channel::Channel<'a, HighSpeed>, BuzzerError> {
        let hstimer0 = {
            let hstim = self.cfg_timer(tone)?;
            self.hstimer.insert(hstim)
        };
        // Initialize LEDC Channel with duty cycle 50%
        let mut channel0: channel::Channel<'a, HighSpeed> = self.ledc.channel(channel::Number::Channel0, self.buzzer.reborrow());
        channel0
            .configure(channel::config::Config {
                timer: hstimer0,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })?;
        Ok(self.channel.insert(channel0))
    }

    pub fn start_tone(&'a mut self, tone: Note) -> Result<Duration, BuzzerError> {
        let chan = self.cfg_channel(tone)?;
        chan.set_duty(50)?;

        let pause_duration = tone.duration_ms / 10; // 10% of duration_ms
        Ok(Duration::from_millis(tone.duration_ms - pause_duration))
    }

    fn end_tone(&'a mut self) {
        if let Some(chan) = self.channel.take() {
            chan.set_duty(0);
        }
    }

    fn play_a_note(&'a mut self, tone: Note) -> Result<(), BuzzerError> {
        let hstimer0 = {
            let hstim = self.cfg_timer(tone)?;
            self.hstimer.insert(hstim)
        };
        // Initialize LEDC Channel with duty cycle 50%
        let mut channel0: channel::Channel<'a, HighSpeed> = self.ledc.channel(channel::Number::Channel0, self.buzzer.reborrow());
        channel0
            .configure(channel::config::Config {
                timer: hstimer0,
                duty_pct: 50,
                pin_config: channel::config::PinConfig::PushPull,
            })?;
        Ok(())
    }

    pub async fn sound(&mut self, tone: Note) {
        let pause_duration = tone.duration_ms / 10; // 10% of duration_ms

        let mut hstimer0: timer::Timer<'static, HighSpeed> = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
        let mut channel0: channel::Channel<'_, HighSpeed> = self.ledc.channel(channel::Number::Channel0, self.buzzer.reborrow());
        let hstimer_cfg = timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: tone.frequency,
        };
        if let Err(e) = hstimer0.configure(hstimer_cfg) {
            error!("problem configuring hstimer. {}", e);
        }
        let channel_cfg = channel::config::Config {
            timer: &hstimer0,
            duty_pct: 50,
            pin_config: channel::config::PinConfig::PushPull,
        };
        if let Err(e) = channel0.configure(channel_cfg){
            error!("problem configuring channel. {}", e);
        }

        Timer::after(Duration::from_millis(tone.duration_ms - pause_duration)).await;

        channel0.set_duty(0).unwrap();
        Timer::after(Duration::from_millis(pause_duration)).await;

    }

    pub async fn play_tone(&mut self, rate: Rate, duration_ms: u64) {
        let pause_duration = duration_ms / 10; // 10% of duration_ms

        let mut hstimer0 = self.ledc.timer::<HighSpeed>(timer::Number::Timer0);
        let mut channel0 = self.ledc.channel(channel::Number::Channel0, self.buzzer.reborrow());
        let hstimer_cfg = timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: rate,
        };
        if let Err(e) = hstimer0.configure(hstimer_cfg) {
            error!("problem configuring hstimer. {}", e);
        }
        let channel_cfg = channel::config::Config {
            timer: &hstimer0,
            duty_pct: 50,
            pin_config: channel::config::PinConfig::PushPull,
        };
        if let Err(e) = channel0.configure(channel_cfg){
            error!("problem configuring channel. {}", e);
        }

        Timer::after(Duration::from_millis(duration_ms - pause_duration)).await;

        channel0.set_duty(0).unwrap();
        Timer::after(Duration::from_millis(pause_duration)).await;

    }

}


pub enum BuzzerCommand {
    Pause,
    Resume,
    Play(Song),
}

async fn sound_process(rx: Receiver<'_, CriticalSectionRawMutex, BuzzerCommand, 3>) {
    loop {        
        match rx.receive().await {
            BuzzerCommand::Pause => continue,
            BuzzerCommand::Resume => todo!(),
            BuzzerCommand::Play(song) => todo!(),
        }
    }
}

pub struct Song {
    whole_note: u32,
    pub notes: Vec<NoteData>,
}

// impl Iterator for Song {
//     type Item;

//     fn next(&mut self) -> Option<Self::Item> {
//         todo!()
//     }
// }

impl Song {
    pub fn new(tempo: u16, some_notes: &[NoteData]) -> Self {
        let whole_note = (60_000 * 4) / tempo as u32;
        let notes: Vec<NoteData> = some_notes.into();
        Self { whole_note, notes }
    }

    fn create_note(self, note: &NoteData) -> Note {
        note.to_note(self.whole_note)
        // let NoteData(freq_hz, duration_type) = note;
        // let note_duration = self.calc_note_duration(*duration_type) as u64;
        // let freq_rate = Rate::from_hz(*freq_hz as u32);
        // Note { frequency: freq_rate, duration_ms: note_duration }
    }

    pub fn calc_note_duration(&self, divider: i16) -> u32 {
        if divider > 0 {
            self.whole_note / divider as u32
        } else {
            let duration = self.whole_note / divider.unsigned_abs() as u32;
            (duration as f64 * 1.5) as u32
        }
    }

    pub async fn play_one_note(&self, note: NoteData, buzzer: &mut Buzzer<'_>) {
        let NoteData(freq_hz, duration_type) = note;
        let note_duration = self.calc_note_duration(duration_type) as u64;
        let freq_rate = Rate::from_hz(freq_hz as u32);
        if freq_hz == REST {
            Timer::after(Duration::from_millis(note_duration)).await;
        } else {
            buzzer.play_tone(freq_rate, note_duration).await
        }
    }

    pub async fn play_on(&self, buzzer: &mut Buzzer<'_>) {
        for note in self.notes.iter() {
            self.play_one_note(*note, buzzer).await
        }
    }

}

