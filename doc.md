doc.md



        // let mut ser_buf = [0u8; 64];

        // match serial.read(&mut ser_buf) {
        //     Ok(count) if count > 0 => {
        //         led.set_low(); // Turn on

        //         match i2c.write(0x42>>1, &[0x12]) {
        //             Ok(_result) => {
        //                 delay(400);
        //                 match i2c.read(0x43>>1, &mut read_buf) {
        //                     Ok(_result) => {
        //                         for &c in &read_buf {
        //                             serial.write(&[c]).unwrap();
        //                         }
        //                     },
        //                     _  => {}
        //                 }
                        
        //             },
        //             _  => {}
        //         }
        //         led.set_high(); // Turn on
        //     }
        //     _ => {}
        // }

# captura um ciclo de VSYNC

```rust
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut ser_buf = [0u8; 64];
        match serial.read(&mut ser_buf) {
            Ok(count) if count > 0 => {
                
                let mut vsync_was_high = false;
                let mut _href_raise = false;
                
                loop {
                    // falling edge of VSYNC signals the 
                    // START OF A FRAME
                    if !vsync_was_high && vsync.is_high() {
                        vsync_was_high = true;
                        led.set_low(); // Turn on
                        serial.write(&[0x70]).unwrap();
                    }
                    
                    if vsync_was_high && vsync.is_low() {
                        vsync_was_high = false;
                        led.set_high(); // Turn on
                        break
                    }
                }
            }
            _ => {}

        }
    }
```