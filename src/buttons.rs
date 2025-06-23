use core::{cell::RefCell, ops::Sub, str};

use {esp_backtrace as _, esp_println as _};

use defmt::info;
use embassy_futures::select::select3;
use embassy_sync::{blocking_mutex::CriticalSectionMutex, pubsub::WaitResult};
use embassy_sync::pubsub::PubSubBehavior;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use embassy_time::{Duration, Instant};
use esp_hal::gpio::{Event, Input, Level};
use esp_hal::ram;
use thiserror_no_std::Error;


#[derive(Debug, Error, defmt::Format)]
pub enum ButtonError {
    PubSubError(#[from] embassy_sync::pubsub::Error),
    NotCreated,
}


/// Debounce delay for button inputs.
pub const BUTTON_DEBOUNCE_DELAY: Duration = Duration::from_millis(10);

/// Duration to recognize a long button press.
pub const LONG_PRESS_DURATION: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, defmt::Format)]
pub enum ButtonEvent {
    Short,
    Long,
}


#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RawButtonEvent {
    Down(Instant),
    Up(Instant),
}

impl Sub<RawButtonEvent> for RawButtonEvent {
    type Output = Duration;

    fn sub(self, rhs: RawButtonEvent) -> Self::Output {
        let inst_self = match self {
            RawButtonEvent::Down(instant) => instant,
            RawButtonEvent::Up(instant) => instant,
        };
        let inst_rhs = match rhs {
            RawButtonEvent::Down(instant) => instant,
            RawButtonEvent::Up(instant) => instant,
        };
        inst_self.duration_since(inst_rhs)
    }
    
}


const BUTTON_CHANNEL_MAX_CAPACITY: usize = 4;
const BUTTON_CHANNEL_MAX_SUBSCRIBERS: usize = 4;
const BUTTON_CHANNEL_MAX_PUBLISHERS: usize = 1;
pub type ButtonPubSubChan = PubSubChannel<CriticalSectionRawMutex, RawButtonEvent, BUTTON_CHANNEL_MAX_CAPACITY, BUTTON_CHANNEL_MAX_SUBSCRIBERS, BUTTON_CHANNEL_MAX_PUBLISHERS>;
pub type ButtonPublisher<'a> = embassy_sync::pubsub::Publisher<'a, CriticalSectionRawMutex, RawButtonEvent, BUTTON_CHANNEL_MAX_CAPACITY, BUTTON_CHANNEL_MAX_SUBSCRIBERS, BUTTON_CHANNEL_MAX_PUBLISHERS>;
pub type ButtonSubscriber<'a> = embassy_sync::pubsub::Subscriber<'a, CriticalSectionRawMutex, RawButtonEvent, BUTTON_CHANNEL_MAX_CAPACITY, BUTTON_CHANNEL_MAX_SUBSCRIBERS, BUTTON_CHANNEL_MAX_PUBLISHERS>;
pub type ButtonDynSubscriber<'a> = embassy_sync::pubsub::DynSubscriber<'a, RawButtonEvent>;


pub struct InputButton<'a>(Input<'a>, &'a str);

impl<'a> InputButton<'a> {
    #[must_use]
    pub const fn new(button: Input<'a>, name: &'a str) -> Self {
        Self(button, name)
    }

    #[inline]
    pub const fn name(&self) -> &'a str {
        self.1
    }

    #[inline]
    pub fn input(&mut self) -> &mut Input<'a> {
        &mut self.0
    }

    #[inline]
    pub fn is_interrupt_set(self) -> bool {
        self.0.is_interrupt_set()        
    }

    pub fn handle(&mut self, ts: Instant) -> RawButtonEvent {
        self.input().clear_interrupt();
        match self.input().level() {
            Level::Low => {
                info!("{} Down: {}", self.name(), ts);
                RawButtonEvent::Down(ts)
            },
            Level::High => {
                info!("{} Up: {}", self.name(), ts);
                RawButtonEvent::Up(ts)
            },
        }            
    }
}


pub struct ButtonReader<'a>(ButtonSubscriber<'a>, &'a str, Option<RawButtonEvent>);

impl<'a> ButtonReader<'a> {
    #[must_use]
    pub const fn new(sub: ButtonSubscriber<'a>, name: &'a str) -> Self {
        Self(sub, name, None)
    }

    #[inline]
    pub fn name(&self) -> &'a str {
        self.1
    }


    pub fn debounce(&self, raw_event: RawButtonEvent) -> RawButtonEvent {
        match self.2 {
            Some(previous) => {
                let btn_elapsed = raw_event - previous;
                if btn_elapsed >= BUTTON_DEBOUNCE_DELAY {
                    raw_event
                } else {
                    previous
                }
            },
            None => raw_event,
        }
    }

    pub async fn read_debounced(&mut self) -> RawButtonEvent {
        loop {
            let wr = self.0.next_message().await;
            if let WaitResult::Message(raw_event) = wr {
                return self.debounce(raw_event)
            }
        }
    }

    pub async fn get_button_event(&mut self) -> ButtonEvent {
        loop {            
            let raw_event = self.read_debounced().await;
            if let Some(previous_event) = self.2 {
                let elapsed = raw_event - previous_event;
                if let (RawButtonEvent::Down(_), RawButtonEvent::Up(_)) = (previous_event, raw_event) {
                    // down followed by up
                    if elapsed >= LONG_PRESS_DURATION {
                        info!("Button({}): Long ({})", self.1, elapsed);
                        return ButtonEvent::Long;
                    } else {
                        info!("Button({}): Short ({})", self.1, elapsed);
                        return ButtonEvent::Short;
                    }
                };
            } else {
                self.2 = Some(raw_event);
            }
        }
    }
}

pub struct ButtonReaderCollection<'a> {
    pub a: ButtonReader<'a>,
    pub b: ButtonReader<'a>,
    pub c: ButtonReader<'a>,    
}

impl<'a> ButtonReaderCollection<'a> {
    pub fn new(a: ButtonReader<'a>, b: ButtonReader<'a>, c: ButtonReader<'a>) -> Self {
        Self { a, b, c }
    }

    pub async fn button_events(&mut self) {
        let a_events = self.a.get_button_event();
        let b_events = self.b.get_button_event();
        let c_events = self.c.get_button_event();
        match select3(a_events, b_events, c_events).await {
            embassy_futures::select::Either3::First(a) => {
                info!("Button A: {}", a);
            },
            embassy_futures::select::Either3::Second(b) => {
                info!("Button B: {}", b);                
            },
            embassy_futures::select::Either3::Third(c) => {
                info!("Button C: {}", c);                
            },
        }
    }
}

#[embassy_executor::task]
pub async fn btn_task(btn_reader: ButtonReaderCollection<'static>) {
    let mut btn_readers = btn_reader;
    loop {
        btn_readers.button_events().await;
    }
}

pub static INPUT_BUTTONS: CriticalSectionMutex<RefCell<Option< ButtonInputCollection<'static> >>> = CriticalSectionMutex::new(RefCell::new(None));


pub static A_CHAN: ButtonPubSubChan = ButtonPubSubChan::new();
pub static B_CHAN: ButtonPubSubChan = ButtonPubSubChan::new();
pub static C_CHAN: ButtonPubSubChan = ButtonPubSubChan::new();


pub struct ButtonInputCollection<'a> {
    pub a: InputButton<'a>,
    pub b: InputButton<'a>,
    pub c: InputButton<'a>,
    pub a_pub: ButtonPublisher<'a>,
    pub b_pub: ButtonPublisher<'a>,
    pub c_pub: ButtonPublisher<'a>,
}

impl<'a> ButtonInputCollection<'a> {
    // #[must_use]
    pub fn new(mut btn_a: Input<'static>, mut btn_b: Input<'a>, mut btn_c: Input<'a>) -> Result<Self, ButtonError> {
        btn_a.listen(Event::AnyEdge);
        btn_b.listen(Event::AnyEdge);
        btn_c.listen(Event::AnyEdge);

        let a: InputButton<'a> = InputButton::new(btn_a, "btn_a");
        let b: InputButton<'a> = InputButton::new(btn_b, "btn_b");
        let c: InputButton<'a> = InputButton::new(btn_c, "btn_c");

        Ok(            
            Self { a, b, c, 
                a_pub: A_CHAN.publisher()?, b_pub: B_CHAN.publisher()?, c_pub: C_CHAN.publisher()?,
            }
        )
    }

    #[ram]
    pub fn interrupt_handler(&mut self) {
        let a = self.a.input();
        let b = self.b.input();
        let c = self.c.input();

        let ts = embassy_time::Instant::now();
        
        if a.is_interrupt_set() {
            let event = self.a.handle(ts);
            self.a_pub.publish_immediate(event);
            info!("Button A");
        }
        if b.is_interrupt_set() {
            let event = self.b.handle(ts);
            self.b_pub.publish_immediate(event);
            info!("Button B");
        }
        if c.is_interrupt_set() {
            let event = self.c.handle(ts);
            self.c_pub.publish_immediate(event);
            info!("Button C");
        }
    }
}


// pub struct Button<'a>(Input<'a>);

// impl<'a> Button<'a> {
//     /// Creates a new `Button` instance.
//     #[must_use]
//     pub const fn new(button: Input<'a>) -> Self {
//         Self(button)
//     }

//     #[inline]
//     async fn wait_for_button_up(&mut self) -> &mut Self {
//         self.0.wait_for_low().await;
//         self
//     }

//     #[inline]
//     async fn wait_for_button_down(&mut self) -> &mut Self {
//         self.0.wait_for_high().await;
//         self
//     }

//     /// Measures the duration of a button press.
//     ///
//     /// This method does not wait for the button to be released.  It only waits
//     /// as long as necessary to determine whether the press was "short" or "long".
//     pub async fn press_duration(&mut self) -> PressDuration {
//         self.wait_for_button_up().await;
//         Timer::after(BUTTON_DEBOUNCE_DELAY).await;
//         self.wait_for_button_down().await;
//         Timer::after(BUTTON_DEBOUNCE_DELAY).await;
//         let press_duration =
//             match select(self.wait_for_button_up(), Timer::after(LONG_PRESS_DURATION)).await {
//                 Either::First(_) => PressDuration::Short,
//                 Either::Second(()) => PressDuration::Long,
//             };
//         info!("Press duration: {:?}", press_duration);
//         press_duration
//     }

//     /// Waits for the button to be pressed.
//     #[inline]
//     pub async fn wait_for_press(&mut self) -> &mut Self {
//         self.0.wait_for_rising_edge().await;
//         self
//     }
// }

// // Instead of having API describing a short vs a long button-press vaguely using a `bool`, we define
// // an `enum` to clarify what each state represents.  The compiler will compile this down to the
// // very same `boolean` that we would have coded by hand.
// #[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, defmt::Format)]
// pub enum PressDuration {
//     Short,
//     Long,
// }
