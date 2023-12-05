import RPi.GPIO as GPIO

PWM_OUTPUT_PIN = 33

p = GPIO.PWM(PWM_OUTPUT_PIN, 50)


def setup_pwm():
    """
    Sets up the PWM output pin.
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PWM_OUTPUT_PIN, GPIO.OUT, initial=GPIO.HIGH)
    p.start(50)


def close_pwm():
    """
    Closes the PWM output pin.
    """
    p.stop()
    GPIO.cleanup()


if __name__ == '__main__':
    setup_pwm()
    while True:
        try:
            duty_cycle = float(input("Enter duty cycle: "))
            p.ChangeDutyCycle(duty_cycle)
        except KeyboardInterrupt:
            break
    close_pwm()
