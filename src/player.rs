//! Music player state machine and command interface for Watchy-M5.
//!
//! This module provides types and an async player for controlling song playback
//! using the buzzer driver. It supports play, pause, stop, and song loading commands,
//! and reacts to button events for playback control.

use core::{cell::RefCell, marker::PhantomData};
use defmt::info;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex};
use static_cell::StaticCell;
use crate::buttons::{ButtonEvent, InputEvent, InputSubscriber, BUTTON_EVENT_CHANNEL};
use crate::{buzzer::BuzzerSender, music::Song};

/// Depth of the player command channel.
const PLAYER_CHANNEL_DEPTH: usize = 3;

/// Embassy channel for sending player commands.
pub type PlayerChannel = Channel::<CriticalSectionRawMutex, PlayerCmd, PLAYER_CHANNEL_DEPTH>;
/// Sender for player commands.
pub type PlayerSender<'a> = embassy_sync::channel::Sender<'a, CriticalSectionRawMutex, PlayerCmd, PLAYER_CHANNEL_DEPTH>;
/// Receiver for player commands.
pub type PlayerReceiver<'a> = embassy_sync::channel::Receiver<'a, CriticalSectionRawMutex, PlayerCmd, PLAYER_CHANNEL_DEPTH>;

/// Global static for the player sender, protected by a critical section mutex.
pub static PLAYER_SEND: CriticalSectionMutex<RefCell<Option< PlayerSender<'static> >>> = CriticalSectionMutex::new(RefCell::new(None));

/// Commands for controlling the music player.
#[derive(Debug, PartialEq)]
pub enum PlayerCmd {
    /// Stop playback.
    Stop,
    /// Pause playback.
    Pause,
    /// Start or resume playback.
    Play,
    /// Load a new song.
    LoadSong(Song<'static>),
}

impl defmt::Format for PlayerCmd {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            PlayerCmd::Stop => defmt::write!(fmt, "Stop"),
            PlayerCmd::Pause => defmt::write!(fmt, "Pause"),
            PlayerCmd::Play => defmt::write!(fmt, "Play"),
            PlayerCmd::LoadSong(_song) => defmt::write!(fmt, "Load"),
        }
    }
}

/// State of the music player.
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub enum PlayerState {
    /// No song loaded.
    #[default] 
    NoSong,
    /// Song loaded and stopped.
    Stopped(Song<'static>),
    /// Song loaded and paused.
    Paused(Song<'static>),
    /// Song loaded and playing.
    Playing(Song<'static>),
}

impl PlayerState {
    /// Returns true if the player is currently playing a song.
    pub fn is_playing(self) -> bool {
        matches!(self, PlayerState::Playing(_))
    }

    /// Returns the currently playing song, if any.
    pub fn get_playing_song(self) -> Option<Song<'static>> {
        match self {
            PlayerState::Playing(song) => Some(song),
            _ => None,
        }
    }

    /// Returns the loaded song, if any, regardless of state.
    pub fn song(self) -> Option<Song<'static>> {
        match self {
            PlayerState::NoSong => None,
            PlayerState::Stopped(song) => Some(song),
            PlayerState::Paused(song) => Some(song),
            PlayerState::Playing(song) => Some(song),
        }
    }

    /// Executes a button input event and returns the new player state.
    ///
    /// Button A/B/C toggles between playing and paused/stopped states.
    pub fn exec_input(self, event: InputEvent) -> PlayerState {
        match (event, self) {
            (_, PlayerState::NoSong) => PlayerState::NoSong,
            (InputEvent::ButtonA(_button_event), PlayerState::Stopped(song)) => PlayerState::Playing(song),
            (InputEvent::ButtonB(_button_event), PlayerState::Stopped(song)) => PlayerState::Playing(song),
            (InputEvent::ButtonC(_button_event), PlayerState::Stopped(song)) => PlayerState::Playing(song),
            (InputEvent::ButtonA(_button_event), PlayerState::Paused(song)) => PlayerState::Playing(song),
            (InputEvent::ButtonB(_button_event), PlayerState::Paused(song)) => PlayerState::Playing(song),
            (InputEvent::ButtonC(_button_event), PlayerState::Paused(song)) => PlayerState::Playing(song),
            (InputEvent::ButtonA(_button_event), PlayerState::Playing(song)) => PlayerState::Paused(song),
            (InputEvent::ButtonB(_button_event), PlayerState::Playing(song)) => PlayerState::Paused(song),
            (InputEvent::ButtonC(_button_event), PlayerState::Playing(song)) => PlayerState::Paused(song),
        }
    }

    /// Executes a player command and returns the new player state.
    pub fn exec_cmd(self, cmd: &mut PlayerCmd) -> PlayerState {
        match (cmd, self) {
            (PlayerCmd::Stop, PlayerState::Paused(mut song)) => {
                song.stop();
                PlayerState::Stopped(song)
            },
            (PlayerCmd::Stop, PlayerState::Playing(mut song)) => {
                song.stop();
                PlayerState::Stopped(song)                
            },
            (PlayerCmd::Stop, PlayerState::Stopped(mut song)) => {
                song.stop();
                PlayerState::Stopped(song)                
            },
            (PlayerCmd::Pause, PlayerState::Paused(song)) => PlayerState::Paused(song),
            (PlayerCmd::Pause, PlayerState::Playing(song)) => PlayerState::Paused(song),
            (PlayerCmd::Pause, PlayerState::Stopped(song)) => PlayerState::Paused(song),
            (PlayerCmd::Play, PlayerState::Stopped(song)) => PlayerState::Playing(song),
            (PlayerCmd::Play, PlayerState::Paused(song)) => PlayerState::Playing(song),
            (PlayerCmd::Play, PlayerState::Playing(song)) => PlayerState::Playing(song),
            (PlayerCmd::Stop, PlayerState::NoSong) => PlayerState::NoSong,
            (PlayerCmd::Pause, PlayerState::NoSong) => PlayerState::NoSong,
            (PlayerCmd::Play, PlayerState::NoSong) => PlayerState::NoSong,
            (PlayerCmd::LoadSong(song), _) => PlayerState::Stopped(*song),
        }
    }
}

/// The music player, which manages playback state and communicates with the buzzer.
///
/// The player receives commands and button events, and sends notes to the buzzer.
pub struct Player<'p> {
    /// Sender for buzzer commands.
    buzz: BuzzerSender<'p>,
    /// Current player state.
    state: PlayerState,
    /// Marker for lifetime.
    _phantom: PhantomData<&'p PlayerChannel>,
    /// Receiver for player commands.
    rx: PlayerReceiver<'p>,
}

/// Static cell for the player command channel.
static PCHAN: StaticCell<PlayerChannel> = StaticCell::new();

impl<'p> Player<'p> {
    /// Creates and returns a static reference to the player command channel.
    pub fn create_chan() -> &'static mut PlayerChannel {        
        PCHAN.init_with(PlayerChannel::new)
    }

    /// Constructs a new `Player` instance.
    ///
    /// # Arguments
    /// * `buzz` - The buzzer command sender.
    /// * `rx` - The player command receiver.
    ///
    /// # Returns
    /// A new `Player`.
    pub fn new(buzz: BuzzerSender<'p>, rx: PlayerReceiver<'p>) -> Self {
        Self { 
            buzz, 
            _phantom: PhantomData, 
            state: PlayerState::default(),
            rx,
        }
    }

    /// Plays the next note in the song and returns the new player state.
    ///
    /// # Arguments
    /// * `song` - The song to play.
    ///
    /// # Returns
    /// `PlayerState::Playing(song)` if there are more notes, or `PlayerState::Stopped(song)` if finished.
    pub async fn playing(&self, mut song: Song<'static>) -> PlayerState {
        match song.play_next_note(self.buzz).await {
            Some(_) => PlayerState::Playing(song),
            None => PlayerState::Stopped(song),
        }
    }

    /// Handles player logic, reacting to both button events and player commands.
    ///
    /// This method concurrently waits for button events, player commands, and note playback,
    /// updating the player state accordingly.
    ///
    /// # Arguments
    /// * `input_sub` - The input event subscriber.
    ///
    /// # Returns
    /// The new player state.
    pub async fn fancy_exec(&mut self, input_sub: &mut InputSubscriber<'p>) -> PlayerState {
        let cmd_rx = self.rx.receive();
        let msg_rx = input_sub.next_message_pure();
        self.state = {
            if let Some(song) = self.state.get_playing_song() {
                let play = self.playing(song);
                match select3(msg_rx, cmd_rx, play).await {
                    Either3::First(event) => self.state.exec_input(event),
                    Either3::Second(mut cmd) => self.state.exec_cmd(&mut cmd),
                    Either3::Third(s) => s,
                }
            } else {
                match select(msg_rx, cmd_rx).await {
                    Either::First(event) => {
                        if let InputEvent::ButtonC(ButtonEvent::Short) = event {
                            info!("{}", esp_alloc::HEAP.stats());
                        }
                        self.state.exec_input(event)
                    },
                    Either::Second(mut cmd) => self.state.exec_cmd(&mut cmd),
                }
            }
        };
        self.state
    }

    /// Handles player commands and advances playback state.
    ///
    /// Waits for a command, executes it, and updates the player state.
    ///
    /// # Returns
    /// The new player state.
    pub async fn exec(&mut self) -> PlayerState {
        let player_rx = self.rx;
        let mut guard = self.state;
        if guard.is_playing() {
            if let Some(song) = guard.song() {
                guard = self.playing(song).await;
            }
        }

        let mut cmd = player_rx.receive().await;
        let player_state = guard.exec_cmd(&mut cmd);
        guard = player_state;
        self.state = guard;
        player_state
    }
}

/// Embassy executor task for running the player event loop.
///
/// This task waits for button events and player commands, and manages playback state.
#[embassy_executor::task(pool_size = 4)]
pub async fn player_task(mut player: Player<'static>) {
    let mut input_event_sub: Option<InputSubscriber<'_>> = BUTTON_EVENT_CHANNEL.subscriber().ok();
    loop {
        if let Some(input_sub) = input_event_sub.as_mut() {
            player.fancy_exec(input_sub).await;
        } else {
            player.exec().await;
        }
    }
}
