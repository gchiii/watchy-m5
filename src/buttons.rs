use core::ops::Sub;

use {esp_backtrace as _, esp_println as _};

use defmt::{info, trace};
use embassy_futures::select::select3;
#[cfg(feature = "gpio_interrupt")]
use embassy_sync::{blocking_mutex::CriticalSectionMutex, pubsub::WaitResult};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use embassy_time::{Duration, Instant};
use esp_hal::gpio::{Event, Input, InputConfig, InputPin, Level, Pull};
#[cfg(feature = "gpio_interrupt")]
use esp_hal::{handler};

use esp_hal::gpio::Io;
#[cfg(feature = "gpio_interrupt")]
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

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, defmt::Format)]
pub enum InputEvent {
    ButtonA(ButtonEvent),
    ButtonB(ButtonEvent),
    ButtonC(ButtonEvent),
}
pub type InputPubSubChan = PubSubChannel<CriticalSectionRawMutex, InputEvent, BUTTON_CHANNEL_MAX_CAPACITY, BUTTON_CHANNEL_MAX_SUBSCRIBERS, BUTTON_CHANNEL_MAX_PUBLISHERS>;
pub type InputPublisher<'a> = embassy_sync::pubsub::Publisher<'a, CriticalSectionRawMutex, InputEvent, BUTTON_CHANNEL_MAX_CAPACITY, BUTTON_CHANNEL_MAX_SUBSCRIBERS, BUTTON_CHANNEL_MAX_PUBLISHERS>;
pub type InputSubscriber<'a> = embassy_sync::pubsub::Subscriber<'a, CriticalSectionRawMutex, InputEvent, BUTTON_CHANNEL_MAX_CAPACITY, BUTTON_CHANNEL_MAX_SUBSCRIBERS, BUTTON_CHANNEL_MAX_PUBLISHERS>;
pub type InputDynSubscriber<'a> = embassy_sync::pubsub::DynSubscriber<'a, InputEvent>;


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


/// Represents a physical input button and its associated metadata.
///
/// `InputButton` wraps an [`Input`] pin and a name for identification. It provides
/// methods to access the pin, check for interrupts, and (optionally) handle GPIO interrupts.
/// This struct is used to abstract the hardware button and provide a consistent interface
/// for button event handling.
pub struct InputButton<'a> { 
    input: Input<'a>, 
    name: &'a str,
}

impl<'a> InputButton<'a> {
    /// Creates a new `InputButton` with the given input pin and name.
    ///
    /// # Arguments
    /// * `button` - The input pin associated with the button.
    /// * `name` - The name of the button.
    ///
    /// # Returns
    /// A new `InputButton` instance.
    #[must_use]
    pub const fn new(button: Input<'a>, name: &'a str) -> Self {
        Self { input: button, name }
    }

    /// Returns the name of the button.
    ///
    /// # Returns
    /// The button name as a string slice.
    #[inline]
    pub const fn name(&self) -> &'a str {
        self.name
    }

    /// Returns a mutable reference to the underlying input pin.
    ///
    /// # Returns
    /// A mutable reference to the [`Input`] pin.
    #[inline]
    pub fn input(&mut self) -> &mut Input<'a> {
        &mut self.input
    }

    /// Checks if an interrupt is set for the button pin.
    ///
    /// # Returns
    /// `true` if an interrupt is set, `false` otherwise.
    #[inline]
    pub fn is_interrupt_set(self) -> bool {
        self.input.is_interrupt_set()        
    }

    /// Handles a GPIO interrupt event and returns the corresponding `RawButtonEvent`.
    ///
    /// This method is only available if the `gpio_interrupt` feature is enabled.
    ///
    /// # Arguments
    /// * `ts` - The timestamp of the interrupt event.
    ///
    /// # Returns
    /// The corresponding [`RawButtonEvent`] (`Down` or `Up`).
    #[cfg(feature = "gpio_interrupt")]
    pub fn interrupt_event(&mut self, ts: Instant) -> RawButtonEvent {
        self.input().clear_interrupt();
        match self.input().level() {
            Level::Low => {
                trace!("{} Down: {}", self.name(), ts);
                RawButtonEvent::Down(ts)
            },
            Level::High => {
                trace!("{} Up: {}", self.name(), ts);
                RawButtonEvent::Up(ts)
            },
        }            
    }

    /// Waits asynchronously for the next button event (edge).
    ///
    /// # Returns
    /// The next [`RawButtonEvent`] (`Down` or `Up`).
    async fn event(&mut self) -> RawButtonEvent {
        match self.input.level() {
            Level::Low => {
                self.input.wait_for_rising_edge().await;
                RawButtonEvent::Up(Instant::now())
            },
            Level::High => {
                self.input.wait_for_falling_edge().await;
                RawButtonEvent::Down(Instant::now())
            },
        }
    }
}



/// A struct that reads and tracks button events from a button subscriber.
///
/// `ButtonReader` provides an interface to receive raw button events,
/// keep track of the previous event, and associate a name with the reader
/// for identification purposes.
///
/// # Fields
/// - `raw_events`: The subscriber that provides raw button events.
/// - `name`: The name associated with this button reader.
/// - `previous_event`: The last received raw button event, if any.
pub struct ButtonReader<'a> { 
    #[cfg(feature = "gpio_interrupt")]
    raw_event_source: ButtonSubscriber<'a>, 
    #[cfg(not(feature = "gpio_interrupt"))]
    btn: InputButton<'a>,
    name: &'a str, 
    previous_event: Option<RawButtonEvent> 
}


impl<'a> ButtonReader<'a> {
    /// Creates a new `ButtonReader` with the given subscriber and name.
    ///
    /// # Arguments
    /// * `sub` - The button event subscriber.
    /// * `name` - The name of the button.
    ///
    /// # Returns
    /// A new `ButtonReader` instance.
    #[cfg(feature = "gpio_interrupt")]
    #[must_use]
    pub const fn new(sub: ButtonSubscriber<'a>, name: &'a str) -> Self {
        Self { raw_event_source: sub, name, previous_event: None }
    }

    #[must_use]
    pub const fn new(btn: InputButton<'a>, name: &'a str) -> Self {
        Self { btn, name, previous_event: None }
    }

    /// Returns the name of the button associated with this reader.
    ///
    /// # Returns
    /// The button name as a string slice.
    #[inline]
    pub fn name(&self) -> &'a str {
        self.name
    }

    /// Waits asynchronously for the next raw button event.
    ///
    /// In interrupt mode (`gpio_interrupt` feature), this method waits for the next
    /// message from the button event subscriber and returns it. In polling mode,
    /// it waits for the next hardware event from the associated `InputButton`.
    ///
    /// # Returns
    /// The next [`RawButtonEvent`] (`Down` or `Up`).
    async fn next_raw_event(&mut self) -> RawButtonEvent {
        #[cfg(feature = "gpio_interrupt")]
        loop {
            let wr = self.raw_event_source.next_message().await;
            if let WaitResult::Message(raw_event) = wr {
                return raw_event;
            }
        }
        #[cfg(not(feature = "gpio_interrupt"))]
        self.btn.event().await
    }

    /// Debounces a raw button event by comparing it to the previous event.
    ///
    /// If the time elapsed since the previous event is greater than or equal to
    /// `BUTTON_DEBOUNCE_DELAY`, the new event is considered valid. Otherwise,
    /// the previous event is returned to filter out spurious presses.
    ///
    /// # Arguments
    /// * `raw_event` - The new raw button event to debounce.
    ///
    /// # Returns
    /// The debounced button event.
    fn debounce_logic(&self, raw_event: RawButtonEvent) -> RawButtonEvent {
        match self.previous_event {
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

    /// Reads the next debounced raw button event asynchronously.
    ///
    /// Waits for the next button event from the subscriber and applies debouncing.
    ///
    /// # Returns
    /// The next debounced `RawButtonEvent`.
    pub async fn read_debounced(&mut self) -> RawButtonEvent {
        let raw_event = self.next_raw_event().await;
        self.debounce_logic(raw_event)
    }

    /// Processes a raw button event and determines if it constitutes a logical button event.
    ///
    /// This function compares the current raw event with the previous event. If the previous event
    /// was a `Down` and the current event is an `Up`, it calculates the elapsed time between them.
    /// If the elapsed time is greater than or equal to `LONG_PRESS_DURATION`, it returns `Some(ButtonEvent::Long)`.
    /// Otherwise, it returns `Some(ButtonEvent::Short)`. If the event sequence does not match a
    /// complete press (i.e., not a `Down` followed by an `Up`), or if there is no previous event,
    /// it returns `None`.
    ///
    /// # Arguments
    /// * `raw_event` - The new raw button event to process.
    ///
    /// # Returns
    /// An `Option<ButtonEvent>` indicating a detected logical button event, or `None` if not applicable.
    fn cook_raw_event(&mut self, raw_event: RawButtonEvent) -> Option<ButtonEvent> {
        let cooked = match self.previous_event {
            Some(previous_raw) => {
                let elapsed = raw_event - previous_raw;
                if let (RawButtonEvent::Down(_), RawButtonEvent::Up(_)) = (previous_raw, raw_event) {
                    // down followed by up
                    if elapsed >= LONG_PRESS_DURATION {
                        trace!("Button({}): Long ({})", self.name, elapsed);
                        Some(ButtonEvent::Long)
                    } else {
                        trace!("Button({}): Short ({})", self.name, elapsed);
                        Some(ButtonEvent::Short)
                    }
                } else {
                    None
                }
            },
            None => {                
                None
            },
        };
        self.previous_event = Some(raw_event);
        cooked
    }

    /// Waits for and returns the next logical button event (`Short` or `Long` press).
    ///
    /// This method tracks the time between button down and up events to determine
    /// if the press was short or long, based on `LONG_PRESS_DURATION`.
    ///
    /// # Returns
    /// The next `ButtonEvent` (`Short` or `Long`).
    pub async fn get_button_event(&mut self) -> ButtonEvent {
        loop {            
            let raw_event = self.read_debounced().await;
            if let Some(cooked) = self.cook_raw_event(raw_event) {
                return cooked;
            }
        }
    }
}

/// A collection of `ButtonReader` instances for multiple buttons.
///
/// `ButtonReaderCollection` groups together readers for three buttons (A, B, and C),
/// allowing unified asynchronous polling for button events. This is useful for
/// applications that want to handle multiple buttons in a coordinated way, such as
/// waiting for any button to be pressed and reporting which one.
///
/// # Fields
/// - `a`: The `ButtonReader` for button A.
/// - `b`: The `ButtonReader` for button B.
/// - `c`: The `ButtonReader` for button C.
pub struct ButtonReaderCollection<'a> {
    pub a: ButtonReader<'a>,
    pub b: ButtonReader<'a>,
    pub c: ButtonReader<'a>,    
}

impl<'a> ButtonReaderCollection<'a> {
    /// Creates a new `ButtonReaderCollection` from three `ButtonReader` instances.
    ///
    /// # Arguments
    /// * `a` - The `ButtonReader` for button A.
    /// * `b` - The `ButtonReader` for button B.
    /// * `c` - The `ButtonReader` for button C.
    ///
    /// # Returns
    /// A new `ButtonReaderCollection` containing the provided readers.
    pub fn new(a: ButtonReader<'a>, b: ButtonReader<'a>, c: ButtonReader<'a>) -> Self {
        Self { a, b, c }
    }

    /// Waits asynchronously for the next logical button event from any button.
    ///
    /// This method concurrently waits for a logical event (`Short` or `Long` press)
    /// from any of the three buttons, and returns an `InputEvent` indicating which
    /// button was pressed and the type of event.
    ///
    /// # Returns
    /// An `InputEvent` representing the next logical event from any button.
    pub async fn button_events(&mut self) -> InputEvent {
        let a_events = self.a.get_button_event();
        let b_events = self.b.get_button_event();
        let c_events = self.c.get_button_event();
        match select3(a_events, b_events, c_events).await {
            embassy_futures::select::Either3::First(a) => {
                trace!("Button A: {}", a);
                InputEvent::ButtonA(a)
            },
            embassy_futures::select::Either3::Second(b) => {
                trace!("Button B: {}", b);
                InputEvent::ButtonB(b)
            },
            embassy_futures::select::Either3::Third(c) => {
                trace!("Button C: {}", c);
                InputEvent::ButtonC(c)
            },
        }
    }
}

#[cfg(feature = "gpio_interrupt")]
#[embassy_executor::task]
pub async fn btn_task(btn_reader: ButtonReaderCollection<'static>) {
    let input_event_pub = BUTTON_EVENT_CHANNEL.publisher().ok();
    let mut btn_readers = btn_reader;
    loop {
        let input_event = btn_readers.button_events().await;
        info!("Event: {}", input_event);
        if let Some(ref iepub) = input_event_pub {
            iepub.publish_immediate(input_event);
        }
    }
}

#[embassy_executor::task]
pub async fn button_dispatch(button_components: ButtonComponents) {
    if let Ok(mut btn_reader) = button_components.build() {
        let input_event_pub = BUTTON_EVENT_CHANNEL.publisher().ok();
        
        loop {
            let input_event = btn_reader.button_events().await;
            info!("Event: {}", input_event);
            if let Some(ref iepub) = input_event_pub {
                iepub.publish_immediate(input_event);
            }
        }
    }
}


#[cfg(feature = "gpio_interrupt")]
pub static INPUT_BUTTONS: CriticalSectionMutex<RefCell<Option< ButtonInputCollection<'static> >>> = CriticalSectionMutex::new(RefCell::new(None));

pub static BUTTON_EVENT_CHANNEL: InputPubSubChan = InputPubSubChan::new();

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
    #[must_use]
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

    pub async fn publish_raw_events(&mut self) {
        let a_raw = self.a.event();
        let b_raw = self.b.event();
        let c_raw = self.c.event();
        
        match select3(a_raw, b_raw, c_raw).await {
            embassy_futures::select::Either3::First(raw_event) => {
                self.a_pub.publish_immediate(raw_event);
                trace!("Button {}", self.a.name());
            },
            embassy_futures::select::Either3::Second(raw_event) => {
                self.b_pub.publish_immediate(raw_event);
                trace!("Button {}", self.b.name());
            },
            embassy_futures::select::Either3::Third(raw_event) => {
                self.c_pub.publish_immediate(raw_event);
                trace!("Button {}", self.c.name());
            },
        }
    }

    #[cfg(feature = "gpio_interrupt")]
    #[ram]
    pub fn interrupt_handler(&mut self) {
        let a = self.a.input();
        let b = self.b.input();
        let c = self.c.input();

        let ts = embassy_time::Instant::now();
        
        if a.is_interrupt_set() {
            let event = self.a.interrupt_event(ts);
            self.a_pub.publish_immediate(event);
            trace!("Button A");
            info!("{}", esp_alloc::HEAP.stats());
        }
        if b.is_interrupt_set() {
            let event = self.b.interrupt_event(ts);
            self.b_pub.publish_immediate(event);
            trace!("Button B");
        }
        if c.is_interrupt_set() {
            let event = self.c.interrupt_event(ts);
            self.c_pub.publish_immediate(event);
            trace!("Button C");
        }
    }
}

#[cfg(feature = "gpio_interrupt")]
pub fn initialize_buttons(button_a: Input<'static>, button_b: Input<'static>, button_c: Input<'static>) -> Result<ButtonReaderCollection<'static>, ButtonError> {
    let btn_inputs = ButtonInputCollection::new(button_a, button_b, button_c)?;

    let a_sub = A_CHAN.subscriber()?;
    let btn_reader_a = ButtonReader::new(a_sub, "btn_a");
    let b_sub = B_CHAN.subscriber()?;
    let btn_reader_b = ButtonReader::new(b_sub, "btn_b");
    let c_sub = C_CHAN.subscriber()?;
    let btn_reader_c = ButtonReader::new(c_sub, "btn_c");


    let btn_readers: ButtonReaderCollection<'static> = ButtonReaderCollection::new(btn_reader_a, btn_reader_b, btn_reader_c);

    critical_section::with(|cs| {
        INPUT_BUTTONS.borrow(cs).replace(Some(btn_inputs));
    });

    Ok(btn_readers)
}

pub struct ButtonComponents {
    pub a: Input<'static>,
    pub b: Input<'static>,
    pub c: Input<'static>,
    pub io: Option<Io<'static>>,
}

impl ButtonComponents {
    pub fn new(a_pin: impl InputPin + 'static, b_pin: impl InputPin + 'static, c_pin: impl InputPin + 'static) -> Self {
        let config = InputConfig::default().with_pull(Pull::Up);
        Self {
            a: Input::new(a_pin, config),
            b: Input::new(b_pin, config),
            c: Input::new(c_pin, config),
            io: None,        
        }
    }

    pub fn with_interrupts(self, io: Io<'static>) -> Self {
        Self {
            io: Some(io),
            ..self
        }
    }

    #[cfg(not(feature = "gpio_interrupt"))]
    pub fn build(self) -> Result<ButtonReaderCollection<'static>, ButtonError> {
        let a = InputButton::new(self.a, "btn_a");
        let b = InputButton::new(self.b, "btn_b");
        let c = InputButton::new(self.c, "btn_c");


        let btn_reader_a = ButtonReader::new(a, "btn_a");
        let btn_reader_b = ButtonReader::new(b, "btn_b");
        let btn_reader_c = ButtonReader::new(c, "btn_c");


        let btn_readers: ButtonReaderCollection<'static> = ButtonReaderCollection::new(btn_reader_a, btn_reader_b, btn_reader_c);

        Ok(btn_readers)
    }

    #[cfg(feature = "gpio_interrupt")]
    pub fn build(self) -> Result<(ButtonInputCollection<'static>, ButtonReaderCollection<'static>), ButtonError> {
        let button_a = self.a;
        let button_b = self.b;
        let button_c = self.c;

        let btn_inputs: ButtonInputCollection<'static> = ButtonInputCollection::new(button_a, button_b, button_c)?;

        let a_sub = A_CHAN.subscriber()?;
        let btn_reader_a = ButtonReader::new(a_sub, "btn_a");
        let b_sub = B_CHAN.subscriber()?;
        let btn_reader_b = ButtonReader::new(b_sub, "btn_b");
        let c_sub = C_CHAN.subscriber()?;
        let btn_reader_c = ButtonReader::new(c_sub, "btn_c");


        let btn_readers: ButtonReaderCollection<'static> = ButtonReaderCollection::new(btn_reader_a, btn_reader_b, btn_reader_c);

        #[cfg(feature = "gpio_interrupt")]
        critical_section::with(|cs| {
            INPUT_BUTTONS.borrow(cs).replace(Some(btn_inputs));
        });

        #[cfg(feature = "gpio_interrupt")]
        if let Some(mut io) = self.io {
            // Set the interrupt handler for GPIO interrupts.
            io.set_interrupt_handler(gpio_interrupt_handler);
        }
        Ok((btn_inputs, btn_readers))
    }
}


#[cfg(feature = "gpio_interrupt")]
#[handler]
#[ram]
fn gpio_interrupt_handler() {
    critical_section::with(|cs| {
        info!("GPIO interrupt");
        if let Some(all_buttons) = INPUT_BUTTONS.borrow(cs).borrow_mut().as_mut() {
            all_buttons.interrupt_handler();
        }
    });
}

