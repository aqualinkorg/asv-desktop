set -e

conan create ./conan-concurrentqueue missionrobotics/stable
conan create ./conan-readerwriterqueue missionrobotics/stable
conan create ./conan-serial missionrobotics/stable
conan create ./mavlink2 missionrobotics/stable
conan create ./mavchannel missionrobotics/stable