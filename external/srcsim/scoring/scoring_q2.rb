#!/usr/bin/env ruby

require 'nokogiri'
require './common'

# Parse an entire log containing states.
class State
  def initialize(file)
    # Open and read the log file
    doc = Nokogiri::XML(File.open(file))

    # Get the first chunk
    chunks = doc.xpath('//gazebo_log/chunk')
    if chunks.size < 2
      puts 'State log file has less than 2 entries.'
      return
    end

    start_line_crossed = false
    finish_line_crossed = false

    @events = {}

    # Create hash of the robot pose over time
    @poses = {}
    for i in 1..chunks.size - 1
      chunk = Nokogiri::XML(chunks[i].text)

      # Read the sim time
      parts = chunk.xpath('//sdf/state/sim_time').text.split
      time = Common::Time.new
      time.sec = parts[0].to_i
      time.nsec = parts[1].to_i

      # Read model pose
      model_pose = Common::Pose.new
      pose = chunk.xpath("//sdf/state/model[@name='valkyrie']/pose")
      next unless pose.size == 1

      parts = pose.text.split
      model_pose.set(parts[0].to_f, parts[1].to_f, parts[2].to_f,
                     parts[3].to_f, parts[4].to_f, parts[5].to_f)

      if model_pose.p.x >= 0.5 && !start_line_crossed
        @events['start_line'] = time
        start_line_crossed = true
      elsif model_pose.p.x >= 4.5 && !finish_line_crossed
        @events['finish_line'] = time
        finish_line_crossed = true
      end
    end
  end

  def event(tag)
    time = Time.new
    found = false
    if @events.key?(tag)
      time = @events[tag]
      found = true
    end

    return found, time
  end
end

# Sanity check: Make sure that the Gazebo log is passed as an argument.
if ARGV.size != 1
  puts 'Usage: scoring.rb <state.log>'
  exit(-1)
end

state_log = ARGV[0]

# Sanity check: Make sure that the Gazebo log exists.
unless File.file?(state_log)
  puts 'Invalid state log file'
  exit(-1)
end

# Read all the state information
state = State.new(state_log)

# Did the robot crossed the starting line?
start_line_crossed, start_time = state.event('start_line')

# Did the robot crossed the finish line?
finish_line_crossed, finish_time = state.event('finish_line')

if start_line_crossed
  printf("Start line crossed: [%d %09d]\n", start_time.sec, start_time.nsec)
else
  printf('Start line crossed: --')
end

if finish_line_crossed
  printf("Finish line crossed: [%d %09d]\n", finish_time.sec, finish_time.nsec)
else
  printf('Finish line crossed: --')
end

if start_line_crossed && finish_line_crossed
  elapsed = finish_time - start_time
  printf("Elapsed time: [%d %09d]\n", elapsed.sec, elapsed.nsec)
else
  printf('Elapsed time: --')
end

exit(0)
