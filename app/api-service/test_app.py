from flask import Flask
import pytest
from app import app, find_place
from unittest.mock import MagicMock, patch

# Create a test client
@pytest.fixture
def client():
    app.config['TESTING'] = True
    with app.test_client() as client:
        yield client

# Mock the googlemaps.Client class
@pytest.fixture
def mock_gmaps():
    with patch('googlemaps.Client') as mock_client:
        yield mock_client

def test_find_place_success(client, mock_gmaps):
    # Mock the places API response
    mock_place_result = {
        'status': 'OK',
        'results': [
            {
                'place_id': 'abc123',
                'geometry': {
                    'location': {
                        'lat': 40.0930404,
                        'lng': -88.2356592
                    }
                }
            }
        ]
    }
    mock_gmaps.return_value.places.return_value = mock_place_result

    # Mock the place details API response
    mock_place_details = {
        'status': 'OK',
        'result': {
            'geometry': {
                'location': {
                    'lat': 40.0930404,
                    'lng': -88.2356592
                }
            }
        }
    }
    mock_gmaps.return_value.place.return_value = mock_place_details

    # Send a request to the find_place endpoint
    response = client.post('/find_place', data={'place_name': 'Highbay'})

    # Assert the response
    assert response.status_code == 200
    assert response.json == { 'lat': 40.0930404,
                        'lng': -88.2356592}

def test_find_place_no_results(client, mock_gmaps):
    # Mock the places API response with no results
    mock_place_result = {
        'status': 'OK',
        'results': []
    }
    mock_gmaps.return_value.places.return_value = mock_place_result

    # Send a request to the find_place endpoint
    response = client.post('/find_place', data={'place_name': 'InvalidPlace'})

    # Assert the response
    assert response.status_code == 200
    assert response.json == {'error': 'No places found'}
