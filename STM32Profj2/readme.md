#ball acceleration simulation

on STM32. using sh1106 & rotary encoder.


**stratege used to draw the ball on sh1106..**
the ball was 8x8 matirx. 
row can be continuously transferred to sh1106, but page(column) is discrete..
So, each column of the ball was shifted according to the position..
![Screenshot from 2025-01-31 11-39-00](https://github.com/user-attachments/assets/a807f0ea-9e80-4ae9-ae7e-498c1e0e8e1a)




**movement of the ball**
ball should accelerate in the direction of encoder, and bounce when facing wall...
![Screenshot from 2025-01-31 11-41-39](https://github.com/user-attachments/assets/35aee37e-328c-4c7e-ab0f-f5d910d38bfd)




