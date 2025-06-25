use core::{cell::RefCell, marker::PhantomData};
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::zerocopy_channel;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex};
use static_cell::StaticCell;
use crate::buttons::{InputEvent, InputSubscriber, BUTTON_EVENT_CHANNEL};
use crate::{buzzer::BuzzerSender, music::Song};



pub type BetterPlayerChannel<'a> = zerocopy_channel::Channel<'a, CriticalSectionRawMutex, PlayerCmd>;
pub type BetterPlayerSender<'a> = zerocopy_channel::Sender<'a, CriticalSectionRawMutex, PlayerCmd>;
pub type BetterPlayerReceiver<'a> = zerocopy_channel::Receiver<'a, CriticalSectionRawMutex, PlayerCmd>;

const PLAYER_CHANNEL_DEPTH: usize = 3;
pub type PlayerChannel = Channel::<CriticalSectionRawMutex, PlayerCmd, PLAYER_CHANNEL_DEPTH>;
pub type PlayerSender<'a> = embassy_sync::channel::Sender<'a, CriticalSectionRawMutex, PlayerCmd, PLAYER_CHANNEL_DEPTH>;
pub type PlayerReceiver<'a> = embassy_sync::channel::Receiver<'a, CriticalSectionRawMutex, PlayerCmd, PLAYER_CHANNEL_DEPTH>;

pub static PLAYER_SEND: CriticalSectionMutex<RefCell<Option< PlayerSender<'static> >>> = CriticalSectionMutex::new(RefCell::new(None));


#[derive(Debug, PartialEq)]
pub enum PlayerCmd {
    Stop,
    Pause,
    Play,
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


#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub enum PlayerState {
    #[default] 
    NoSong,
    Stopped(Song<'static>),
    Paused(Song<'static>),
    Playing(Song<'static>),
}

impl PlayerState {
    pub fn is_playing(self) -> bool {
        matches!(self, PlayerState::Playing(_))
    }

    pub fn get_playing_song(self) -> Option<Song<'static>> {
        match self {
            PlayerState::Playing(song) => Some(song),
            _ => None,
        }
    }

    pub fn song(self) -> Option<Song<'static>> {
        match self {
            PlayerState::NoSong => None,
            PlayerState::Stopped(song) => Some(song),
            PlayerState::Paused(song) => Some(song),
            PlayerState::Playing(song) => Some(song),
        }
    }

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


// the player needs to have a channel for receiving commands and data, and needs a channel to send data to the buzzer
pub struct Player<'p> {
    buzz: BuzzerSender<'p>,
    state: PlayerState,
    _phantom: PhantomData<&'p PlayerChannel>,
    rx: PlayerReceiver<'p>,
}

static PCHAN: StaticCell<PlayerChannel> = StaticCell::new();

impl<'p> Player<'p> {

    pub fn create_chan() -> &'static mut PlayerChannel {        
        PCHAN.init_with(PlayerChannel::new)
    }

    pub fn new(buzz: BuzzerSender<'p>, rx: PlayerReceiver<'p>) -> Self {
        Self { 
            buzz, _phantom: PhantomData, 
            state: PlayerState::default(),
            rx,
        }
    }


    pub async fn playing(&self, mut song: Song<'static>) -> PlayerState {
        match song.play_next_note(self.buzz).await {
            Some(_) => PlayerState::Playing(song),
            None => PlayerState::Stopped(song),
        }
    }

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
                    Either::First(event) => self.state.exec_input(event),
                    Either::Second(mut cmd) => self.state.exec_cmd(&mut cmd),
                }
            }
        };
        self.state
    }

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
