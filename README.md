# Demo-2: Master Branch
Robot must perform two different tasks. Each task is assigned to a team member(s) based on their roles on the team (Raspberry Pi or Arduino).
* **Task 1**: Robot must be able to move within 1 foot of the beacon and stop before hitting it.
* **Task 2**: Robot must move within 1 foot of the beacon, and then must circle the beacon, moving no more than 2 feet away from the beacon at any time.

A short description of files in this master branch are as followed:
* **CLControlDemo2v1.ino**: Receives data from the Raspberry Pi over I2C communication, and utilizes a state machine (swtich statement) to implement the different stages of both Task 1 and Task 2. 
* **FILENAME**: description
