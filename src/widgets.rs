
extern crate alloc;
use core::marker::PhantomData;

use alloc::string::{String, ToString};
use defmt::info;
use embedded_layout::{align::{horizontal, vertical, Align}, View};
use embedded_text::{alignment::HorizontalAlignment, style::{HeightMode, TextBoxStyle, TextBoxStyleBuilder}, TextBox};
use esp_alloc as _;

use embedded_graphics::{mono_font::{ascii::FONT_9X15, MonoTextStyle}, prelude::*, primitives::{PrimitiveStyle, Rectangle, StrokeAlignment, StyledDrawable}, text::{renderer::{CharacterStyle, TextRenderer}, Text}};
use u8g2_fonts::{fonts::u8g2_font_6x12_m_symbols, U8g2TextStyle};


#[derive(Clone, Debug)]
pub enum MyCharacterStyle<'a, C: PixelColor + Default> {
    Mono(MonoTextStyle<'a, C>),
    U8g2(U8g2TextStyle<C>)
}

impl<'a, C: PixelColor + Default> TextRenderer for MyCharacterStyle<'a, C> {
    type Color = C;

    fn draw_string<D>(
        &self,
        text: &str,
        position: Point,
        baseline: embedded_graphics::text::Baseline,
        target: &mut D,
    ) -> Result<Point, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.draw_string(text, position, baseline, target),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.draw_string(text, position, baseline, target),
        }
    }

    fn draw_whitespace<D>(
        &self,
        width: u32,
        position: Point,
        baseline: embedded_graphics::text::Baseline,
        target: &mut D,
    ) -> Result<Point, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.draw_whitespace(width, position, baseline, target),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.draw_whitespace(width, position, baseline, target),
        }
    }

    fn measure_string(&self, text: &str, position: Point, baseline: embedded_graphics::text::Baseline) -> embedded_graphics::text::renderer::TextMetrics {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.measure_string(text, position, baseline),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.measure_string(text, position, baseline),
        }
        
    }

    fn line_height(&self) -> u32 {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.line_height(),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.line_height(),
        }
    }
}

impl<'a, C: PixelColor + Default> TextRenderer for &MyCharacterStyle<'a, C> {
    type Color = C;

    fn draw_string<D>(
        &self,
        text: &str,
        position: Point,
        baseline: embedded_graphics::text::Baseline,
        target: &mut D,
    ) -> Result<Point, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.draw_string(text, position, baseline, target),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.draw_string(text, position, baseline, target),
        }
    }

    fn draw_whitespace<D>(
        &self,
        width: u32,
        position: Point,
        baseline: embedded_graphics::text::Baseline,
        target: &mut D,
    ) -> Result<Point, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.draw_whitespace(width, position, baseline, target),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.draw_whitespace(width, position, baseline, target),
        }
    }

    fn measure_string(&self, text: &str, position: Point, baseline: embedded_graphics::text::Baseline) -> embedded_graphics::text::renderer::TextMetrics {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.measure_string(text, position, baseline),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.measure_string(text, position, baseline),
        }
        
    }

    fn line_height(&self) -> u32 {
        match self {
            MyCharacterStyle::Mono(mono_text_style) => mono_text_style.line_height(),
            MyCharacterStyle::U8g2(u8g2_text_style) => u8g2_text_style.line_height(),
        }
    }
}

impl<'a, C: PixelColor + Default> From<U8g2TextStyle<C>> for MyCharacterStyle<'a, C> {
    fn from(v: U8g2TextStyle<C>) -> Self {
        Self::U8g2(v)
    }
}

impl<'a, C: PixelColor + Default> From<MonoTextStyle<'a, C>> for MyCharacterStyle<'a, C> {
    fn from(v: MonoTextStyle<'a, C>) -> Self {
        Self::Mono(v)
    }
}

impl<'a, C: PixelColor + Default> Default for MyCharacterStyle<'a, C> {
    fn default() -> Self {
        let style = U8g2TextStyle::new(u8g2_font_6x12_m_symbols, C::default());
        MyCharacterStyle::U8g2(style)
    }
}

impl<'a, C: PixelColor + Default> CharacterStyle for MyCharacterStyle<'a, C> {
    type Color = C;
}

impl<'a, C: PixelColor + Default> CharacterStyle for &MyCharacterStyle<'a, C> {
    type Color = C;
}

#[derive(Clone, Debug)]
pub struct TextWindow<'a, C: PixelColor + Default> {
    text: String,
    boundary_box: Rectangle,
    character_style: MyCharacterStyle<'a, C>, // MonoTextStyle<'a, C>,
    textbox_style: TextBoxStyle,
    border_style: PrimitiveStyle<C>,
    _phantom: PhantomData<&'a C>,
}


impl<'a, C: PixelColor + Default> Default for TextWindow<'a, C> {
    fn default() -> Self {
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)            
            .alignment(HorizontalAlignment::Justified)
            .paragraph_spacing(3)
            .build();

        Self { 
            text: String::new(),
            boundary_box: Default::default(), 
            character_style: Default::default(),
            textbox_style,
            border_style: Default::default(),
            _phantom: PhantomData,
        }
    }
}


impl<'a, C: PixelColor + Default> TextWindow<'a, C> {
    pub fn new(top_left: Point, size: Size, character_style: MyCharacterStyle<'a, C> ) -> Self {
        let boundary_box = Rectangle::new(top_left, size);
        Self {
            character_style,
            boundary_box,
            ..Default::default()
        }
    }

    pub fn from_rect(r: Rectangle) -> Self {
        Self {
            boundary_box: r,
            ..Default::default()
        }
    }

    pub fn move_to(&mut self, top_left: Point) {
        self.boundary_box.top_left = top_left;
    }
    
    pub fn border_style_mut(&mut self) -> &mut PrimitiveStyle<C> {
        &mut self.border_style
    }
    
    pub fn border_style(&self) -> PrimitiveStyle<C> {
        self.border_style
    }

    pub fn with_background_color(self, bg_color: C) -> Self {
        let mut character_style = self.character_style;
        let mut border_style = self.border_style;
        border_style.fill_color = Some(bg_color);
        character_style.set_background_color(Some(bg_color));
        Self {
            character_style,
            border_style,
            ..self
        }
    }

    pub fn set_background_color(&mut self, bg_color: C) {
        self.character_style.set_background_color(Some(bg_color));
    }
}

pub trait StyleableTextWindow<'a, C: PixelColor + Default>: Sized {
    fn set_character_style(&mut self, style: MyCharacterStyle<'a, C>);
    fn with_character_style(self, style: MyCharacterStyle<'a, C>) -> Self;

    fn set_textbox_style(&mut self, style: TextBoxStyle);
    fn with_textbox_style(self, style: TextBoxStyle) -> Self;

    fn set_border_style(&mut self, style: PrimitiveStyle<C>);
    fn with_border_style(self, style: PrimitiveStyle<C>) -> Self where Self: Sized {
        let mut win = self;
        win.set_border_style(style);
        win
    }


    fn set_text(&mut self, text: impl ToString);
    fn with_text(self, text: impl ToString) -> Self {
        let mut win = self;
        win.set_text(text);
        win
    }

}

impl<'a, C: PixelColor + Default> StyleableTextWindow<'a, C> for TextWindow<'a, C> {
    fn with_character_style(self, style: MyCharacterStyle<'a, C>) -> Self {
        Self {
            character_style: style,
            ..self
        }
    }

    fn with_textbox_style(self, style: TextBoxStyle) -> Self {
        Self {
            textbox_style: style,
            ..self
        }
    }

    fn with_border_style(self, style: PrimitiveStyle<C>) -> Self {
        Self {
            border_style: style,
            ..self
        }
    }

    fn with_text(self, text: impl ToString) -> Self {
        Self {
            text: text.to_string(),
            ..self
        }
    }
    
    fn set_character_style(&mut self, style: MyCharacterStyle<'a, C>) {
        self.character_style = style;
    }
    
    fn set_textbox_style(&mut self, style: TextBoxStyle) {
        self.textbox_style = style;
    }
    
    fn set_border_style(&mut self, style: PrimitiveStyle<C>) {
        self.border_style = style;
    }
    
    fn set_text(&mut self, text: impl ToString) {
        self.text = text.to_string();
    }

}


impl<'a, C: PixelColor + Default> View for TextWindow<'a, C> {
    fn translate_impl(&mut self, by: Point) {
        View::translate_impl(&mut self.boundary_box, by);
    }

    fn bounds(&self) -> Rectangle {
        self.boundary_box
    }
}

impl<'a, C: PixelColor + Default> Drawable for TextWindow<'a, C> {
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color> 
    {
        let bb = self.boundary_box;
        bb.draw_styled(&self.border_style, target)?;
        let cs = &self.character_style;
        let mut tb = TextBox::with_textbox_style(&self.text, bb, cs, self.textbox_style);
        tb.align_to_mut(&self.boundary_box, horizontal::Center, vertical::Center);
        tb.draw(target)?;
        let mut style = self.border_style;
        style.fill_color = None;
        style.stroke_alignment = StrokeAlignment::Inside;

        bb.draw_styled(&style, target)?;
        Ok(())
    }
}

pub struct ScrollingMarquee<'a, C: PixelColor + Default> {
    text_window: TextWindow<'a, C>,
    boundary_box: Rectangle,
    translation: Point,
    border_style: PrimitiveStyle<C>,
    // scroll_size: Size,
}

impl<'a, C: PixelColor + Default> Default for ScrollingMarquee<'a, C> {
    fn default() -> Self {
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)            
            .alignment(HorizontalAlignment::Left)
            .paragraph_spacing(1)
            .build();

        let text = "Hello World ^_^;";
        let character_style: MyCharacterStyle<'_, C> = MonoTextStyle::<'a, C>::new(&FONT_9X15, C::default()).into();
        let scroll_size = Text::new(&text, Point::zero(), &character_style).bounding_box().size + Size::new_equal(2);

        let text_window = TextWindow::from_rect(Rectangle::new(Point::zero(), scroll_size))
            .with_textbox_style(textbox_style)
            .with_character_style(character_style)
            .with_text(text);
        Self {
            text_window,
            boundary_box: Default::default(),
            translation: Default::default(),
            border_style: Default::default(),
            // scroll_size, 
        }
    }
}

impl<'a, C: PixelColor + Default> ScrollingMarquee<'a, C> {
    pub fn new(top_left: Point, size: Size, font: &'a embedded_graphics::mono_font::MonoFont<'a>, text_color: C) -> Self {
        let boundary_box = Rectangle::new(top_left, size);
        Self::from_rect(boundary_box, font, text_color)
    }

    pub fn from_rect(r: Rectangle, font: &'a embedded_graphics::mono_font::MonoFont<'a>, text_color: C) -> Self {
        let boundary_box = r;
        info!("outer_rect: {}", boundary_box);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)            
            .alignment(HorizontalAlignment::Left)
            .paragraph_spacing(1)
            .build();
        let text = "Hello World ^_^;";
        let character_style: MyCharacterStyle<'_, C> = MonoTextStyle::new(font, text_color).into();
        let scroll_size = Text::new(&text, Point::zero(), &character_style).bounding_box().size + Size::new_equal(2);
        let bounds = Rectangle::new(boundary_box.top_left, scroll_size).align_to(&boundary_box, horizontal::Center, vertical::Center);
        info!("text_box: {}", bounds);
        let text_window = TextWindow::from_rect(bounds)
            .with_textbox_style(textbox_style)
            .with_character_style(character_style)
            .with_text(text);
        
        Self {
            boundary_box,
            text_window,
            // scroll_size,
            ..Default::default()
        }
    }


    fn euclid_wrap(point: Point, rectangle: &Rectangle) -> Point {
        let origin = rectangle.top_left;
        let offset = point - origin;
        let size = rectangle.size;

        // Wrap x-coordinate
        let x = (offset.x).rem_euclid(size.width as i32 + 1) + origin.x;

        // Wrap y-coordinate
        let y = (offset.y).rem_euclid(size.height as i32 + 1) + origin.y;

        Point { x, y }
    }

    pub fn rotate(&mut self) {
        if !(self.translation == Point::zero()) {
            let r = self.boundary_box; 
            let mut bounds = self.text_window.boundary_box;
            let virtual_origin = r.top_left - bounds.size;
            let virtual_space = Rectangle::new(virtual_origin, r.size + bounds.size);
            let current = bounds.top_left;
            let wrapped = Self::euclid_wrap(current + self.translation, &virtual_space);
            bounds.top_left = wrapped;
            self.text_window.move_to(bounds.top_left);
        }
    }
    
    pub fn set_translation(&mut self, translation: Point) {
        self.translation = translation;
    }
    
    pub fn translation(&self) -> Point {
        self.translation
    }
    
    pub fn boundary_box(&self) -> Rectangle {
        self.boundary_box
    }
    
    pub fn text_window_mut(&mut self) -> &mut TextWindow<'a, C> {
        &mut self.text_window
    }
    
    pub fn text_window(&self) -> &TextWindow<'a, C> {
        &self.text_window
    }
    
    pub fn set_text_window(&mut self, text_window: TextWindow<'a, C>) {
        self.text_window = text_window;
    }
    
    pub fn set_border_style(&mut self, border_style: PrimitiveStyle<C>) {
        self.border_style = border_style;
    }
    
    pub fn border_style(&self) -> PrimitiveStyle<C> {
        self.border_style
    }
    
    pub fn border_style_mut(&mut self) -> &mut PrimitiveStyle<C> {
        &mut self.border_style
    }
}

impl<'a, C: PixelColor + Default> StyleableTextWindow<'a, C> for ScrollingMarquee<'a, C> {
    fn with_character_style(self, style: MyCharacterStyle<'a, C>) -> Self {
        let text_window = self.text_window.with_character_style(style);
        Self {
            text_window,
            ..self
        }
    }

    fn with_textbox_style(self, style: TextBoxStyle) -> Self {
        let text_window = self.text_window.with_textbox_style(style);
        Self {
            text_window,
            ..self
        }
    }

    fn with_border_style(self, style: PrimitiveStyle<C>) -> Self {
        let text_window = self.text_window.with_border_style(style);
        Self {
            text_window,
            ..self
        }
    }

    fn with_text(self, text: impl ToString) -> Self {
        let text_window = self.text_window.with_text(text);
        Self {
            text_window,
            ..self
        }
    }
    
    fn set_character_style(&mut self, style: MyCharacterStyle<'a, C>) {
        self.text_window.set_character_style(style);
    }
    
    fn set_textbox_style(&mut self, style: TextBoxStyle) {
        self.text_window.set_textbox_style(style);
    }
    
    fn set_border_style(&mut self, style: PrimitiveStyle<C>) {
        self.text_window.set_border_style(style);
    }
    
    fn set_text(&mut self, text: impl ToString) {
        self.text_window.set_text(text);
    }
}

impl<'a, C: PixelColor + Default> Drawable for ScrollingMarquee<'a, C> {
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = C> 
    {
        self.boundary_box.draw_styled(&self.border_style, target)?;
        self.text_window.draw(target)?;
        
        let mut style = self.border_style;
        style.fill_color = None;
        style.stroke_alignment = StrokeAlignment::Inside;

        let bb = self.boundary_box;
        bb.draw_styled(&style, target)?;

        Ok(())
    }
}
