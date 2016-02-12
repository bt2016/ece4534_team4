# ece4534_team4 Source Code Repo
Source code for ECE4534 Spring2016 Team4

NOTE: You can also configure and manage the git repo via command line. The .git folder is in the C:\microchip\harmony\v1_06_02\apps\ece4534_team4 folder.

### Managing include directories
1. ProjectProperties/General/SourceFolders
 * ../../../../../framework
 * ../../../../../third_party
 * ../src
2. ProjectProperties/Conf:[default]/xc32(GlobalOptions)/xc32-as
 * ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX
 * ../../../../../third_party/rtos/FreeRTOS/Source/include
 * ../src/system_config/default
3. Start MPLAB Harmony Configurator and follow the prompt to choose a new harmony framework
 * Use path C:/microchip/harmony/v1_06_02
 * Click the Generate Code button so that the relative addressed can be re-mapped
4. Check for multiple instances of app.h app.c and main.c
 * if multiple copies exist, then delete them
5. Build the project!

### How to import this project into your MPLAB:
1. Clone the repository: git clone https://github.com/bt2016/ece4534_team4.git C:\microchip\harmony\v1_06_02\apps\ece4534_team4
2. Open MPLAB and open the project. The actual MPLAB project is called ece4534_team4.X in the ece4534_team4/firmware folder
3. Create and switch to a new branch. Go to Team/Branch/Create Branch and Switch to Branch
4. Start working!

### How to commit:
1. Clean the project: Right click on the project icon in the projects window and click 'Clean'
2. Left click on the project icon in the projects window to highlight it. Go to Team/Commit. Write a commit message
3. Push to the Github: Go to Team/Remote/Push and select the branch you were working on. Commit the branch

### Other things:
1. How to check what branches there are: git branch -v
2. How to check what remotes are available: git remote -v

