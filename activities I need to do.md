activities I need to do:



after slowing the rate at which the clock goes

```C


&nbsp;   // In the CTL register, set the TASSEL and ID bits:

&nbsp;   //  - \[x] Choose SMCLK as timer clock source (TASSEL = 10b)

&nbsp;   //  - \[ ] Choose prescale value of 4 (ID = 2); 2 << 6

&nbsp;   //  - \[x] Choose prescale value of 8 (ID = 3); 3 << 6

&nbsp;   TIMER\_A1->CTL  |=  0x0200;

//    TIMER\_A1->CTL  |=  0x0080;

&nbsp;   TIMER\_A1->CTL  |=  0x00C0;
```

the driver no longer funcitons as intended. it freezes. perhaps it's part of the other issue I encountered. fix it so that it doesn't update as fast.

also I am getting small angle sweeps of about 1.5 radians. fix it so that there's a capture of 6.28 radians. adjust .skip_factor and OUTPUT_BUFFER and also:

#define TASK_4_OFFSET   5

shift more time if the collection is taking longer or shift less time if data processing is taking longer!

pay attention to:

#ifdef DEBUG_OUTPUTS
            printf("am here 6");
            printf("%5d\n", local_counter);
#endif

feel free to ask chatgpt now that I know that it prints 14 times after crapping out

```
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
am here 6    2
```
