use core::cell::RefCell;

use embassy_sync::{blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex}, channel::Channel};

use crate::{buzzer::BuzzerSender, music::Song};



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
    LoadSong(&'static mut Song<'static>),
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


#[derive(Debug, Default, PartialEq)]
pub enum PlayerState {
    #[default] 
    NoSong,
    Stopped(&'static mut Song<'static>),
    Paused(&'static mut Song<'static>),
    Playing(&'static mut Song<'static>),
}

impl PlayerState {
    pub fn exec_cmd(self, cmd: PlayerCmd) -> PlayerState {
        match (cmd, self) {
            (PlayerCmd::Stop, PlayerState::Paused(song)) => {
                song.stop();
                PlayerState::Stopped(song)
            },
            (PlayerCmd::Stop, PlayerState::Playing(song)) => {
                song.stop();
                PlayerState::Stopped(song)                
            },
            (PlayerCmd::Stop, PlayerState::Stopped(song)) => {
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
            (PlayerCmd::LoadSong(song), _) => PlayerState::Stopped(song),
        }
    }
}


#[embassy_executor::task(pool_size = 4)]
pub async fn player_task(player_rx: PlayerReceiver<'static>, buzzer_tx: BuzzerSender<'static>) {
    // song.play(buzzer_tx).await
    let mut player_state = PlayerState::default();

    loop {
        if player_state == PlayerState::NoSong {
            let cmd = player_rx.receive().await;
            player_state = player_state.exec_cmd(cmd);
        }
        player_state = match player_state {
            PlayerState::NoSong => PlayerState::NoSong,
            PlayerState::Stopped(song) => PlayerState::Stopped(song),
            PlayerState::Paused(song) => PlayerState::Paused(song),
            PlayerState::Playing(song) => {
                song.play(buzzer_tx).await;
                PlayerState::Playing(song)
            },
        };
        let cmd = player_rx.receive().await;
        player_state = player_state.exec_cmd(cmd);

        // if let PlayerState::Playing(song) = player_state {
        //     let play_future = song.play(buzzer_tx);
        // }

        // if player_state == PlayerState::NoSong {
        //     let cmd = player_rx.receive().await;
        //     player_state = player_state.exec_cmd(cmd);
        // } else {
        // }
    }
    
}
