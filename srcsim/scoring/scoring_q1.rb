#!/usr/bin/env ruby

require 'nokogiri'
require 'matrix'
require './common'

# Calculate position distance between two matrices
def transformPositionDistance(matA, matB)
  a = Vector[matA[0, 3],
             matA[1, 3],
             matA[2, 3]]

  b = Vector[matB[0, 3],
             matB[1, 3],
             matB[2, 3]]

  return (a - b).magnitude()
end

# Get a matrix from pose elements
def transformFromPose(x, y, z, roll, pitch, yaw)

  m00 = Math.cos(yaw) * Math.cos(pitch)
  m01 = Math.cos(yaw) * Math.sin(pitch) * Math.sin(roll) - Math.sin(yaw) * Math.cos(roll)
  m02 = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) + Math.sin(yaw) * Math.sin(roll)
  m03 = x

  m10 = Math.sin(yaw) * Math.cos(pitch)
  m11 = Math.sin(yaw) * Math.sin(pitch) * Math.sin(roll) + Math.cos(yaw) * Math.cos(roll)
  m12 = Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll) - Math.cos(yaw) * Math.sin(roll)
  m13 = y

  m20 = -Math.sin(pitch)
  m21 = Math.cos(pitch) * Math.sin(roll)
  m22 = Math.cos(pitch) * Math.cos(roll)
  m23 = z

  return Matrix[
           [m00, m01, m02, m03],
           [m10, m11, m12, m13],
           [m20, m21, m22, m23],
           [0.0, 0.0, 0.0, 1.0]
         ]
end

# Get a matrix from a chunk and a path
def getTransform(chunk, path)
  pose = chunk.xpath(path)

  if pose.size != 1
    printf("Couldn't find pose for path %s\n", path)
    return
  end

  parts = pose.text().split
  if parts.size != 6
    printf("Malformed pose for path %s\n", path)
    return
  end

  return transformFromPose(parts[0].to_f,
                     parts[1].to_f,
                     parts[2].to_f,
                     parts[3].to_f,
                     parts[4].to_f,
                     parts[5].to_f)
end

class State
  def initialize(file)
    # Open and read the log file
    doc = Nokogiri::XML(File.open(file))

    # Get the first chunk
    chunks = doc.xpath("//gazebo_log/chunk")
    if chunks.size < 2
      puts "State log file has less than 2 entries."
      return
    end

    chunk = Nokogiri::XML(chunks[0].text)

    # Get console pose in world frame
    tConsoleWorld = getTransform(chunk, "//sdf/world/model[@name='console1']/pose")

    # printf("Console world pos [%f %f %f]\n",
    #        tConsoleWorld[0, 3], tConsoleWorld[1, 3], tConsoleWorld[2, 3])

    # Read all the light positions
    @tLightWorld = Hash.new
    for i in 1..44

      # Light in console frame (local frame because this is not from states)
      tLightConsole = getTransform(chunk, "//sdf/world/model/link/visual[@name='light#{i}']/pose")

      # Light in world frame
      @tLightWorld[i] = tConsoleWorld * tLightConsole

      # printf("Light [%i] world position [%f %f %f]\n", i,
      #        *@tLightWorld[i][0, 3], *@tLightWorld[i][1, 3], *@tLightWorld[i][2, 3])
    end

    # Create hash of the neck pose over time
    @tNeckWorld = Hash.new
    for i in 1..chunks.size-1
      chunk = Nokogiri::XML(chunks[i].text)

      # Read the sim time
      parts = chunk.xpath("//sdf/state/sim_time").text.split
      time = Common::Time.new
      time.sec = parts[0]
      time.nsec = parts[1]

      # Read the neck pose in world frame (world frame because they're from states)
      @tNeckWorld[time] = getTransform(chunk, "//sdf/state/model/link[@name='upperNeckPitchLink']/pose")

      # printf("Time[%d.%d] Pose[%f %f %f]\n", time.sec, time.nsec,
      #         @tNeckWorld[time][0, 3], @tNeckWorld[time][1, 3], @tNeckWorld[time][2, 3])
    end
  end

  # Return the matrix of a light in the world frame, according to the index
  def lightMat(index)

    @tLightWorld.each do |key, mat|
      if key == index
        return mat
      end
    end

    printf("Light [%i] not found. Returning identity matrix.\n", index)
    return Matrix.identity(4)
  end

  # Return the last matrix of the neck in the world frame before the given time
  def neckMat(time)

    @tNeckWorld.each_with_index do |(key, mat), index|

      if key >= time
        prevKey = @tNeckWorld.keys[index-1]
        # printf("Time wanted[%d.%d] Time found[%d.%d]\n", time.sec, time.nsec, prevKey.sec, prevKey.nsec)
        return @tNeckWorld[prevKey]
      end
    end

    # Last element
    if (time >= @tNeckWorld.keys.last)
      return @tNeckWorld[@tNeckWorld.keys.last]
    end

    printf("Neck pose for time [%d.%d] not found. Returning identity matrix.\n", time.sec, time.nsec)
    return Matrix.identity(4)
  end
end

if ARGV.size != 2
  puts "Usage: scoring.rb <answer.log> <state.log>"
  exit -1
end

qualLog = ARGV[0]
stateLog = ARGV[1]

if !File.file?(qualLog)
  puts "Invalid qual1 log file"
  exit 0
end

if !File.file?(stateLog)
  puts "Invalid state log file"
  exit 0
end

black = Common::Color.new

# Read all the state information
state = State.new(stateLog)

# Start time
start = Common::Time.new

# Latest registered time
latestTime = Common::Time.new

# Latest registered color
latestColor = Common::Color.new

# Latest registered light pose in the world frame
tLightWorldLatest = Matrix.identity(4)

# Keep track of how many answers have been processed
answerCount = 1

# Sum of euclidean error for all colors
colorTotalError = 0

# Sum of euclidean error for all positions (neck)
neckTotalError = 0

# Sum of euclidean error for all positions (head)
headTotalError = 0

# Sum of euclidean error for all positions (flipped head frame)
headFlippedTotalError = 0

# Transform from neck (upperNeckPitchLink) to head
tHeadNeck = transformFromPose(0.183585961, 0.0, 0.075353826, -3.14159, 0.130899694, 0.0)

File.open(qualLog).each do |line|

  # Skip lines that begin with "#"
  if line =~ /^#/
    next
  end

  # Split the line
  parts = line.split

  # Process the "start" line
  if line =~ /^start/
    if parts.size != 3
      puts "Invalid 'start' line, exiting: "
      puts line
      exit 0
    end

    start.sec = parts[1].to_i
    start.nsec = parts[2].to_i
  end

  # Process the "switch" line
  if line =~ /^switch/
    if parts.size != 8
      puts "Invalid 'switch' line, exiting: "
      puts line
      exit 0
    end

    latestColor.r = parts[2].to_f
    latestColor.g = parts[3].to_f
    latestColor.b = parts[4].to_f
    latestColor.a = parts[5].to_f

    lightTime = Common::Time.new
    lightTime.sec = parts[6].to_i
    lightTime.nsec = parts[7].to_i
    latestTime = lightTime

    # If not black, then set the light index
    if latestColor != black
      lightIndex = parts[1].to_i
      tLightWorldLatest = state.lightMat(lightIndex)

      # printf("Switch: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Light world pos[%6.4f %6.4f %6.4f] Index[%d]\n",
      #        lightTime.sec + lightTime.nsec * 1e-9,
      #        latestColor.r, latestColor.g, latestColor.b,
      #        tLightWorldLatest[0, 3], tLightWorldLatest[1, 3], tLightWorldLatest[2, 3],
      #        lightIndex)
    end
  end

  # Process the "answer" line
  if line =~ /^answer/
    if parts.size != 9
      puts "Invalid 'answer' line, exiting: "
      puts line
      exit 0
    end

    # Answer time
    answerTime = Common::Time.new
    answerTime.sec = parts[7].to_i
    answerTime.nsec = parts[8].to_i
    latestTime = answerTime

    ##### COLOR #####

    # Answer color
    answerColor = Common::Color.new
    answerColor.r = parts[4].to_f
    answerColor.g = parts[5].to_f
    answerColor.b = parts[6].to_f

    # Compute difference to previous light color
    colorError = answerColor.difference(latestColor)
    colorTotalError += colorError

    ##### POSITION #####

    # Answer pose (could be in neck or head frame)
    tLightAnswer = transformFromPose(parts[1].to_f, parts[2].to_f, parts[3].to_f, 0, 0, 0)

    ##### NECK POSITION #####

    # Neck matrix in world frame at this time
    tNeckWorld = state.neckMat(answerTime)
    tWorldNeck = tNeckWorld.inverse()

    # Light pose in neck frame - ground truth
    # light -> world -> neck
    tLightNeck = tWorldNeck * tLightWorldLatest

    # Compute distance between the light pose and the answer
    neckError = transformPositionDistance(tLightNeck, tLightAnswer)
    neckTotalError += neckError

    ##### HEAD POSITION #####

    # Head matrix in world frame at this time
    tHeadWorld = tNeckWorld * tHeadNeck
    tWorldHead = tHeadWorld.inverse()

    # Light pose in head frame - ground truth
    # light -> world -> neck -> head
    tLightHead = tWorldHead * tLightWorldLatest

    # Compute distance between the light pose and the answer
    headError = transformPositionDistance(tLightHead, tLightAnswer)
    headTotalError += headError

    ##### HEAD POSITION FLIPPED #####

    # Rotate image by pi about head's X axis (flip image upside-down)
    rollPi = transformFromPose(0, 0, 0, Math::PI, 0, 0)
    tLightHeadFlipped = rollPi * tLightHead

    headFlippedError = transformPositionDistance(tLightHeadFlipped, tLightAnswer)
    headFlippedTotalError += headFlippedError

    # Print answer summary
    printf("Answer %i: Color:    answer                         [%2.4f %2.4f %2.4f]\n",
           answerCount, answerColor.r, answerColor.g, answerColor.b)

    printf("                    ground truth                   [%2.4f %2.4f %2.4f]\n",
           latestColor.r, latestColor.g, latestColor.b)

    printf("                    euclidean error                [%2.6f]\n", colorError)

    printf("          Position: answer                         [%2.4f %2.4f %2.4f]\n",
           tLightAnswer[0, 3], tLightAnswer[1, 3], tLightAnswer[2, 3])

    printf("                    ground truth (neck)            [%2.4f %2.4f %2.4f]\n",
           tLightNeck[0, 3], tLightNeck[1, 3], tLightNeck[2, 3])

    printf("                    ground truth (head)            [%2.4f %2.4f %2.4f]\n",
           tLightHead[0, 3], tLightHead[1, 3], tLightHead[2, 3])

    printf("                    ground truth (head flipped)    [%2.4f %2.4f %2.4f]\n",
           tLightHeadFlipped[0, 3], tLightHeadFlipped[1, 3], tLightHeadFlipped[2, 3])

    printf("                    euclidean error (neck)         [%2.6f]\n", neckError)

    printf("                    euclidean error (head)         [%2.6f]\n", headError)

    printf("                    euclidean error (head flipped) [%2.6f]\n", headFlippedError)

    answerCount += 1
  end
end

# Calculate duration
duration = Common::Time.new
duration = latestTime - start

printf("--------------------------------\n")
printf("Duration: %d.%d\n", duration.sec, duration.nsec)
printf("Total color euclidean error:                   %1.6f\n", colorTotalError)
printf("Total position euclidean error (neck):         %1.6f\n", neckTotalError)
printf("Total position euclidean error (head):         %1.6f\n", headTotalError)
printf("Total position euclidean error (head flipped): %1.6f\n", headFlippedTotalError)
printf("--------------------------------\n")

