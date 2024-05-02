from flask import Flask, render_template, request, jsonify
import googlemaps
from dotenv import load_dotenv
import os
# Load the environment variables from the .env file
load_dotenv()
api_key = os.getenv('GOOGLE_MAPS_API_KEY')

app = Flask(__name__)


gmaps = googlemaps.Client(key=api_key)

@app.route('/')
def index():
    return render_template('index.html', api_key=api_key)

@app.route('/find_place', methods=['POST'])
def find_place():
    place_name = request.form['place_name']
    
    # Search for places based on the input name
    places_result = gmaps.places(query=place_name)

    if places_result['status'] == 'OK' and len(places_result['results']) > 0:
        # Get the first (best-matched) place from the search results
        place = places_result['results'][0]

        # Extract the place ID
        place_id = place['place_id']

        # Retrieve the details of the place using the place ID
        place_details = gmaps.place(place_id=place_id)

        if place_details['status'] == 'OK':
            # Extract the latitude and longitude coordinates
            lat = place_details['result']['geometry']['location']['lat']
            lng = place_details['result']['geometry']['location']['lng']
            print(lat, lng)
            # Return the coordinates as JSON
            return jsonify({'lat': lat, 'lng': lng})
        else:
            return jsonify({'error': 'Unable to retrieve place details'})
    else:
        return jsonify({'error': 'No places found'})

if __name__ == '__main__':
    app.run(debug=True)