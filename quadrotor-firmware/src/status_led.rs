use core::future::{Future, Pending, pending};
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio;
use embassy_time::{Duration, Timer};

use crate::datatypes::{ControllerState, ControllerStateSignal};

const INACTIVE_BLINK_INTERVAL: Duration = Duration::from_millis(500);

// TODO: Is there a better way to do this?
enum TimerOrPending {
    Timer(Timer),
    Pending(Pending<()>),
}

impl Future for TimerOrPending {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        match core::pin::Pin::get_mut(self) {
            Self::Timer(timer) => {
                let repinned = core::pin::pin!(timer);
                repinned.poll(cx)
            },
            Self::Pending(pending_) => {
                let repinned = core::pin::pin!(pending_);
                repinned.poll(cx)
            },
        }
    }
}

#[embassy_executor::task]
pub async fn status_led_task(mut led: gpio::Output<'static>, controller_state_signal: &'static ControllerStateSignal) -> ! {
    // Retrieve first state
    let mut controller_state = controller_state_signal.wait().await;

    loop {
        let blink_future = match controller_state {
            ControllerState::Inactive => TimerOrPending::Timer(Timer::after(INACTIVE_BLINK_INTERVAL)),
            ControllerState::Active => TimerOrPending::Pending(pending()),
        };
        let controller_state_change_future = controller_state_signal.wait();

        // Wait for next blink interval or controller state change
        match select(blink_future, controller_state_change_future).await {
            Either::First(_) => led.toggle(),
            Either::Second(ControllerState::Inactive) => {
                led.set_low();
                controller_state = ControllerState::Inactive;
            },
            Either::Second(ControllerState::Active) => {
                led.set_high();
                controller_state = ControllerState::Active;
            },
        }
    }
}