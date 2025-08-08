//! Music note definitions, song representation, and playback logic for Watchy-M5.
//!
//! This module provides note frequency constants, note and song types, and
//! conversion utilities for playing music using the buzzer driver.

use crate::{buzzer::{BuzzerCommand, BuzzerSender}, player::PlayerCmd};

use {esp_backtrace as _, esp_println as _};

use core::marker::PhantomData;

use embassy_time::{Duration, Instant, Timer};
use esp_hal::time::Rate;

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

/// Represents a note length in music notation (e.g., quarter, eighth, dotted).
///
/// The absolute value is the divisor (1 = whole, 2 = half, etc).
/// The sign indicates if the note is dotted (negative = dotted).
#[derive(Clone, Copy)]
pub struct NoteLength (pub i16);

impl NoteLength {
    /// Returns true if the note is dotted.
    pub fn is_dotted(&self) -> bool {
        let NoteLength(l) = self;
        *l < 0
    }
    /// Calculates the duration in milliseconds for this note length.
    ///
    /// # Arguments
    /// * `whole_note` - The duration of a whole note in ms.
    ///
    /// # Returns
    /// The duration in milliseconds.
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

/// Type alias for music note duration (notation divisor).
pub type MusicNoteDuration = i16;
/// Type alias for music note frequency in Hertz.
pub type MusicNoteHertz = f64;
/// Type alias for buzzer note duration in milliseconds.
pub type BuzzerNoteDuration = u32;

/// Represents a note or rest for the buzzer, with frequency and duration.
#[derive(Clone, Copy, Debug, defmt::Format, PartialEq)]
pub enum BuzzerNote {
    /// A rest (silence) for the given duration.
    Rest(BuzzerNoteDuration),
    /// A sound at the given frequency and duration.
    Sound(Rate, BuzzerNoteDuration),
}

/// Represents a music note or rest, using frequency and notation duration.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum MusicNote {
    /// A rest (silence) for the given duration (notation divisor).
    Rest(MusicNoteDuration),
    /// A sound at the given frequency and duration (notation divisor).
    Sound(MusicNoteHertz, MusicNoteDuration),
}

impl MusicNote {
    /// Calculates the duration in milliseconds for a note.
    ///
    /// # Arguments
    /// * `divider` - The note length divisor.
    /// * `whole_note` - The duration of a whole note in ms.
    ///
    /// # Returns
    /// The duration in milliseconds.
    fn duration_ms(divider: &MusicNoteDuration, whole_note: &u32) -> u32 { 
        if *divider > 0 {
            whole_note / *divider as u32
        } else {
            let duration = whole_note / divider.unsigned_abs() as u32;
            (duration as f64 * 1.5) as u32
        }
    }

    /// Converts a `MusicNote` to a `BuzzerNote` using the given whole note duration.
    ///
    /// # Arguments
    /// * `whole_note` - The duration of a whole note in ms.
    ///
    /// # Returns
    /// The corresponding `BuzzerNote`.
    pub fn to_buzzer_note(&self, whole_note: &u32) -> BuzzerNote {
        match self {
            MusicNote::Rest(d) => {
                BuzzerNote::Rest(Self::duration_ms(d, whole_note))
            },
            MusicNote::Sound(freq_hz, d) => {
                let rate = Rate::from_hz(*freq_hz as u32);
                let duration_ms = Self::duration_ms(d, whole_note);
                BuzzerNote::Sound(rate, duration_ms)
            },
        }
    }
}

impl From<(MusicNoteHertz, MusicNoteDuration)> for MusicNote {
    /// Converts a tuple of (frequency, duration) into a `MusicNote`.
    fn from(val: (MusicNoteHertz, MusicNoteDuration)) -> Self {
        let (hertz, duration) = val;
        if hertz == REST {
            MusicNote::Rest(duration)
        } else {
            MusicNote::Sound(hertz, duration)
        }
    }
}

/// Represents a note with a frequency and duration for the buzzer.
#[derive(Clone, Copy, Debug, defmt::Format, PartialEq)]
pub struct Note {
    /// Frequency of the note.
    pub frequency: Rate, 
    /// Duration of the note in milliseconds.
    pub duration_ms: u64,
}

/// Represents a song as a sequence of notes and playback state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Song<'s> {
    /// Duration of a whole note in milliseconds.
    whole_note: u32,
    /// Slice of notes in the song.
    pub notes: &'s [MusicNote],
    /// Current index in the song.
    index: usize,
    /// Marker for lifetime.
    _phantom: PhantomData<&'s MusicNote>,
}

impl Song<'_> {
    /// Creates a new `Song` from a tempo and a note slice.
    ///
    /// # Arguments
    /// * `tempo` - The song tempo in beats per minute.
    /// * `some_notes` - The slice of notes for the song.
    ///
    /// # Returns
    /// A new `Song` instance.
    pub fn new(tempo: u16, some_notes: &'static [MusicNote]) -> Self {
        let whole_note = (60_000 * 4) / tempo as u32;
        Self { whole_note, notes: some_notes, _phantom: PhantomData, index: 0 }
    }

    /// Advances to the next note and returns it as a `BuzzerNote`.
    ///
    /// # Returns
    /// `Some(BuzzerNote)` if there is a next note, or `None` if at the end.
    pub fn next_note(&mut self) -> Option<BuzzerNote>{
        if let Some(note) = self.notes.get(self.index) {
            let buz_note = note.to_buzzer_note(&self.whole_note);
            self.index += 1;
            return Some(buz_note);
        }
        None
    }

    /// Plays the next note asynchronously, sending it to the buzzer.
    ///
    /// # Arguments
    /// * `tx` - The `BuzzerSender` to send buzzer commands.
    ///
    /// # Returns
    /// `Some(index)` if a note was played, or `None` if at the end.
    pub async fn play_next_note(&mut self, tx: BuzzerSender<'_>) -> Option<usize> {
        if let Some(buz_note) = self.next_note() {
            let now = Instant::now();
            let duration_ms = match buz_note {
                BuzzerNote::Rest(duration_ms) => duration_ms,
                BuzzerNote::Sound(rate, duration_ms) => {
                    tx.send(BuzzerCommand::Sound(BuzzerNote::Sound(rate, duration_ms))).await;
                    duration_ms
                },
            } as u64;
            let expires_at = now + Duration::from_millis_floor(duration_ms as u64);
            Timer::at(expires_at).await;
            Some(self.index)
        } else {
            self.index = 0;
            None
        }
    }

    /// Plays the entire song asynchronously, sending each note to the buzzer.
    ///
    /// # Arguments
    /// * `tx` - The `BuzzerSender` to send buzzer commands.
    pub async fn play(&mut self, tx: BuzzerSender<'_>) {
        let note_iter = self.notes.iter();
        for note in note_iter {
            let buz_note = note.to_buzzer_note(&self.whole_note);
            let duration_ms = match buz_note {
                BuzzerNote::Rest(duration_ms) => duration_ms,
                BuzzerNote::Sound(rate, duration_ms) => {
                    tx.send(BuzzerCommand::Sound(BuzzerNote::Sound(rate, duration_ms))).await;
                    duration_ms
                },
            } as u64;
            Timer::after(Duration::from_millis(duration_ms)).await;
        }
    }

    /// Stops playback and resets the song index.
    pub fn stop(&mut self) {
        self.index = 0;
    }
}

/// Represents the playback state of a song.
#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum SongState {
    /// Song is stopped.
    #[default] 
    Stopped,
    /// Song is paused.
    Paused,
    /// Song is playing.
    Playing,
}

impl SongState {
    /// Executes a player command and returns the new state.
    ///
    /// # Arguments
    /// * `cmd` - The player command to execute.
    ///
    /// # Returns
    /// The new `SongState`.
    pub fn execute(self, cmd: PlayerCmd) -> Self {        
        match (self, cmd) {
            (SongState::Stopped, PlayerCmd::Play) => SongState::Playing,
            (SongState::Paused, PlayerCmd::Stop) => SongState::Stopped,
            (SongState::Paused, PlayerCmd::Play) => SongState::Playing,
            (SongState::Playing, PlayerCmd::Stop) => SongState::Stopped,
            (SongState::Playing, PlayerCmd::Pause) => SongState::Paused,
            (state, _) => state,
        }
    }
}


