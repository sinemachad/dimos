
## Goals
We are building this (seriously - exactly this) https://www.youtube.com/watch?v=Zkj5WSae3Uc

We are designing a human-centric interface, not an agentic interface for now.
Good human centric interface allows us to test our own tooling before giving it to an agent,

Reason for this is that if you give something directly to an agent without using it yourself, it might be shit and agent might be rightfully underperforming.

## all sensor data is stored by default for every run
    auto rotation/cleanup, set max_size

## Data streams

- These are sensor streams (all sensor data is stored by default for every run)
- But also other data streams created in real time or async.

### All datapoints that have a temporal index (are `Timestamped`)

for all temporal datapoints we need to be able to get spatial info for - this is important for multi embodiment, we do this either by having robot_id associated to the stream,

robots can see the same shoe from different angles, we can deduplicate once temporal or spatial matches are there

or by directly storing a 4D index or time + 3D index. how we actually store stuff is not important and storage system dependent, how we query is what we care about.

so I can quickly ask for a video frame, where/when it was captured.
I can detect on top of it, fetch a related LIDAR frame, project

### Search

Different datastreams provide different types of search, text or image

Facial recognition datastream can accept a face search, time or space
Agent narration or Video stream can search by vector embedding, time or space
Sound recording, search by time or space

Some of these abilities imply other types of search, being able to accept embedding search means you can search by text or by image as well

## Reprocessing, parallel streams

different algos can create different new datastreams (like embedding models for example, LLM narration etc)
some of these datastreams are slower than realtime, with ability to catch up (like embeddings aren't generated if robot isn't moving) some of these are to be stored permanently, some are temporary and part of some analysis and will be thrown away.

if this is designed well, on API level we don't care if we are dealing with a stored stream or a search result, we don't care if stream is stored (and where) or in memory as part of some analysis etc.

### Example

speaker clustering model is analyzing audio, gives speaker embedding stream (with temporal/spatial index)
correlating facial recognition embeddings to speech embeddings we can match a face to voice

## Semantic Costmaps

overlay semantic similarity onto a costmap rendered in rerun in realtime

## Object Search

We have many frames to analyze with VLM, analysis is costly (but cheaper if batched!)
So we need to use traditional search algos, use semantic similarity as a heuristic, find hotspots in time and space to analyze with VLMs (just some standard hill climbing, simulated annealing and such. keep in mind we might not be looking for a global optimum but local hills) we can also use clustering algos etc

Once best matches are found, project into 3d

## logs

system logs, human-agent-tool interaction are also temporal/textual streams that can be stored, embedded, searched over

### Embedding data streams

# milestone 1

I can query for "a shoe" in a textbox, get a semantic map overlay

# milestone 2

I can query for "a shoe" in a textbox, get PointStamped for shoes detected by VLM

## example interaction 1: memory search

search for "a shoe" - independent stored streams offer textual queries

3 agent narration matches (temporal textual stream2)
1 tool call match (temporal textual stream 2)
temporal-semantic graph returned (image embeddings)

temporal-spatial-semantic graph analysis - 3 clusters identified, feed each cluster to some description VLM - "a shoe on a kitchen floor", "a shoe on a desk" etc

return to an agent:

- narration block, timestamp
- tool call match, timestamp
- return 3 clusters, timestamps, rough locations

agent calls goto (event cluster 3)

cluster 3 - find best image, correlate to lidar, project into space, navigate, once there, use VLM and visual nav

## example interaction 2: arm

mustafa is able to ask for an object in proximity to the robot. robot searches memory biasing distance in time and space. if close match is not found, search can be expanded

"do you remember the red sock"

"yes I saw it 35 seconds ago"

"yes I saw it 3 days ago behind me"

"yes I saw it an hour ago, it was 15 meters away"


# Questions

"where was I, when this log line was added"

"how long for have I been observing this object"
