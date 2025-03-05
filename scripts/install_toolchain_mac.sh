git clone https://github.com/wpilibsuite/allwpilib.git
cd allwpilib
./gradlew installRoborioToolchain

echo 'export FRCHOME=~/wpilib/2025' >> ~/.zshrc
echo 'export WPILIB_HOME=~/wpilib/2025' >> ~/.zshrc
echo 'export PATH=$FRCHOME/roborio/bin:$PATH' >> ~/.zshrc
source ~/.zshrc