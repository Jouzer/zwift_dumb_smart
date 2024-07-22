# zwift_dumb_smart
A hobby project to create a Wemos/Arduino device that can make a cheap dumb trainer a smart one in Zwift
Zwift communicates with some expensive smart trainer devices the resistance that you "should" be feeling according to the simulation (ascension grade, current speed, drafting someone etc.) . The smart trainer then increases or decreases resistance you feel at the pedals.
My cheap trainer didn't have this capability, but it had a "trainer resistance" lever which pulled a wire. One day I thought, shouldn't it be possible to create a device that pulls/pushes this wire to mimic the behaviour..?

I wasn't the first to think about it, I found Pete Everetts guide later that day and went with my budget from there.

This is an altered version of https://www.instructables.com/Zwift-Interface-for-Dumb-Turbo-Trainer/ by Pete Everett
I kept his Bluetooth code but I rewrote other parts of the code to accomodate for my budget version. I think mine ended up costing somewhere between 50-100€ minus the trainer which I had already. 

The project worked, and I rode half a winter using it, I recall occasional glitching. My Tacx trainer soon started to show too much wear and instead of polishing the project I went for a Zwift Hub commercial solution.

**Parts I used:**
- Tacx Satori Smart. Essentially any dumb trainer works that has a "trainer resistance" lever. The lever needs to be the type where it pulls a cable
- Plastic housing with aluminium endplates, 30cm x 15cm x 10cm
- 24V DC PSU
- L298N motor driver board
- Stepper motor from AliExpress https://a.aliexpress.com/_EjRUSAx
- LOLIN32 Wifi / Bluetooth development board
- 24VDC to 5VDC adapter for L298N
- Then some microswitches, LEDs, resistors and potentiometers from my local electronics store

<img src="https://github.com/user-attachments/assets/269eadf8-c38b-419c-9e01-dda8177742b2" alt="Finished build" width="200" />
</br>

Finished end product. Green signals Bluetooth status, yellow is lit when motor moves (or should be moving), red is lit when the motor has ran into a microswitch. Then there's manual switch if I wanted to ride without Zwift, also 2 potentiometers to adjust the scale (meaning the full range of the cable wire could mean anything from -10%..0% gradient to 5%..15% gradient. It's nice to feel a change on a downhill, something like -1%..7% was good

</br>
<img src="https://github.com/user-attachments/assets/7c4903e7-5234-4fd5-a5c5-0d4d7d925449" alt="Picture from the inside" width="200" />
Quite stuffed inside
