//! Example using the OV7670 camera module without FIFO

#![deny(unsafe_code)]
#![no_std]
#![no_main]

// use panic_halt as _;
use panic_semihosting as _;

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use num_format::{Buffer, Locale};

use nb::block;

use stm32f1xx_hal::{
    serial::{Config, Serial},
    spi::{Mode as ModeSpi, Phase, Polarity, Spi},
    pac, prelude::*,
    delay::Delay as Delay2,
    timer::{Tim2NoRemap, Timer},
    i2c::{BlockingI2c, Mode, DutyCycle},
    // pwm::Channel,

};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, Ssd1306};
// use cortex_m_semihosting::hprintln;


#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(56.mhz())
        .pclk1(28.mhz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();

    let mut gpioa = dp.GPIOA.split();
    
    // TIM2
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    // let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    // If you don't want to use all channels, just leave some out
    // let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);
    let pins = (c1, c2);

    let mut pwm = Timer::tim2(dp.TIM2, &clocks)
        .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 10000.khz())
        .split();

    pwm.0.enable();
    pwm.0.set_duty(pwm.0.get_max_duty()/2);
    

    let mut gpioc = dp.GPIOC.split();
    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);    

    let mut gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let mut i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
        // Mode::Standard {
            frequency: 100_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    // USART1
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let serial = Serial::usart1(
        dp.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
    );


    // Display
    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let mut delay2 = Delay2::new(cp.SYST, clocks);

    let mut rst = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    let dc = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        ModeSpi {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        8.mhz(),
        clocks,

    );

    let interface = display_interface_spi::SPIInterfaceNoCS::new(spi, dc);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.reset(&mut rst, &mut delay2).unwrap();
    display.init().unwrap();

    
    let vsync = gpioa.pa2.into_pull_up_input(&mut gpioa.crl);
    let href = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);
    let pclk = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);

    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);


    let d0 = gpiob.pb7.into_pull_up_input(&mut gpiob.crl);
    let d1 = gpiob.pb6.into_pull_up_input(&mut gpiob.crl);
    let d2 = gpiob.pb5.into_pull_up_input(&mut gpiob.crl);
    let d3 = pb4.into_pull_up_input(&mut gpiob.crl);
    let d4 = pb3.into_pull_up_input(&mut gpiob.crl);
    let d5 = pa15.into_pull_up_input(&mut gpioa.crh);
    let d6 = gpioa.pa12.into_pull_up_input(&mut gpioa.crh);
    let d7 = gpioa.pa11.into_pull_up_input(&mut gpioa.crh);

    
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut read_buf = [70u8; 1];

    delay(1400);
    

    // IF we want a 24 MHz PCLK. The only possible configuration is prescaler by 2, and PLL x6.
    // CLKRC Bit[6] must be 0, to enable prescaler.
    // CLKRC Bit[0-5] must be 1, to enable prescaler by 2.
    match i2c.write(0x42>>1, &[0x11, 0b00011111]) {
    // match i2c.write(0x42>>1, &[0x11, 0b00001111]) { //faster
        Ok(_result) => {
            delay(400);
            match i2c.read(0x43>>1, &mut read_buf) {
                Ok(_result) => {
                    delay(100);
                    let mut buf = Buffer::new();
                    buf.write_formatted(&(read_buf[0]), &Locale::en);
                    Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top).draw(&mut display).unwrap();
                },
                _  => {}
            }
            
        },
        _  => {}
    }

    // DBLV Bit[7-6] must be 10, if to enable PLL x6    
    match i2c.write(0x42>>1, &[0x6B, 0b11001010]) {
        Ok(_result) => {
            delay(400);
            match i2c.read(0x43>>1, &mut read_buf) {
                Ok(_result) => {
                    delay(400);
                    let mut buf = Buffer::new();
                    buf.write_formatted(&(read_buf[0]), &Locale::en);
                    Text::with_baseline(&buf, Point::new(20, 0), text_style, Baseline::Top).draw(&mut display).unwrap();
                },
                _  => {}
            }
            
        },
        _  => {}
    }


    // say we want to use the QCIF format, we'll need to enable the scaling, and select the QCIF format.
    // COM3 Bit[3] must be 1, to enable scaling
    match i2c.write(0x42>>1, &[0x0C, 0b00001000]) {
    // match i2c.write(0x42>>1, &[0x0C]) {
        Ok(_result) => {
            delay(400);
            match i2c.read(0x43>>1, &mut read_buf) {
                Ok(_result) => {
                    delay(400);
                    let mut buf = Buffer::new();
                    buf.write_formatted(&(read_buf[0]), &Locale::en);
                    Text::with_baseline(&buf, Point::new(40, 0), text_style, Baseline::Top).draw(&mut display).unwrap();
                },
                _  => {}
            }
            
        },
        _  => {}
    }

    // COM7 Bit[3] must be 1, to use the QCIF format
    match i2c.write(0x42>>1, &[0x12, 0b00001000]) {
        Ok(_result) => {
            delay(400);
            match i2c.read(0x43>>1, &mut read_buf) {
                Ok(_result) => {
                    delay(400);
                    let mut buf = Buffer::new();
                    buf.write_formatted(&(read_buf[0]), &Locale::en);
                    Text::with_baseline(&buf, Point::new(60, 0), text_style, Baseline::Top).draw(&mut display).unwrap();
                },
                _  => {}
            }
            
        },
        _  => {}
    }


    delay(400);


    display.flush().unwrap();

    let (mut tx, mut rx) = serial.split();

    let mut flat_frame = [0u8; 64*128];
    let mut pclk_count;

    let pixel_val = || {
        let mut val = 0x00;
        if d0.is_high() {val = val | 0b00000001};
        if d1.is_high() {val = val | 0b00000010};
        if d2.is_high() {val = val | 0b00000100};
        if d3.is_high() {val = val | 0b00001000};
        if d4.is_high() {val = val | 0b00010000};
        if d5.is_high() {val = val | 0b00100000};
        if d6.is_high() {val = val | 0b01000000};
        if d7.is_high() {val = val | 0b10000000};
        return val
    };

    loop {

        match block!(rx.read()) {
            Ok(_c) => {
                pclk_count = 0;

                // falling edge of VSYNC signals the
                // START OF A FRAME
                loop {
                    if vsync.is_high() {break;}
                }
                loop {
                    if vsync.is_low() {break;}
                }

                for y in 0..64 {
                    // rising edge of HREF signals the start of a line,
                    // and the falling edge of HREF signals the end of the line.
                    // D0-D7 must be sampled only when HREF is high
                    // At this point it shoud be low, so we wait it to rise
                    loop {
                        if href.is_high() {break;}
                    }
                    led.set_low();
                    for x in 0..128 {
                        // at this moment pclk shoud be low or falling
                        // loop {if pclk.is_low() {break;}}
                        
                        // D0-D7 must be sampled at the rising edge of the PCLK signal
                        // ignore ch
                        loop {if pclk.is_high() {break;}}
                        // let val = pixel_val();
                        loop {if pclk.is_low() {break;}}
                        // ignore ch
                        loop {if pclk.is_high() {break;}}
                        // let val = pixel_val();
                        loop {if pclk.is_low() {break;}}
                        
                        // take ch 1
                        loop {if pclk.is_high() {break;}}
                        let val = pixel_val();
                        if val > 180 {display.set_pixel(x, 64-y, true);} else {display.set_pixel(x, 64-y, false);}                        
                        loop {if pclk.is_low() {break;}}
                        flat_frame[pclk_count] = val;
                        pclk_count = pclk_count + 1;
                        
                        

                    }
                }
                led.set_high();
                display.flush().unwrap();

                for p in flat_frame {
                    block!(tx.write(p)).ok();
                }

            }
            Err(_) => {
                led.set_high();
                delay(400);
                led.set_low();
            }
        }

    }
}