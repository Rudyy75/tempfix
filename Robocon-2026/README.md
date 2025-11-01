# Robocon-2026 :)))) 🍩🍩🍩
Repository for maintaining all code for ABU Robocon 2026 written by team CSD Robocon NITK. 

## Guidelines
- Work on separate branches, do not push anything to the ```main``` branch.
- While creating a branch follow the following naming convention:
    - If it is a normal task, name it ```Task_<TaskName>```
    - If it is for prototyping a mechanism, name it ```Proto_<MechanismName>```
    - If it is for the final bot, name it ```<Bot(R1 or R2)>```
- Keep the code clean, structured and documented, make sure to explain your code in comments.
- Organize all related code into directories.
- Make sure to ``` git pull ``` before starting your work everyday and ``` git push ``` when you end your day. Solve all merge conflicts immediately, don't leave them for later.

## Pull-Push Guide  
- Clone the repo (if not already):
    - ```git clone git@github.com:csd-robocon-nitk/Robocon-2026.git```
    - ```cd Robocon-2026```
- Switch to your team branch:
    - ```git fetch origin```
    - ```git checkout wall_bot```
- Pull the latest changes:
    - ```git pull origin wall_bot```
- Copy your files to be pushed into YOUR folder.
- Check that files are inside:
    - ```ls YOUR_FOLDER_NAME/```
- Stage YOUR folder only:
    - ```git add YOUR_FOLDER_NAME/```
- Commit changes:
    - ```git commit -m "Commit Nigga"```
- Push into branch:
    - ```git push origin wall_bot```
