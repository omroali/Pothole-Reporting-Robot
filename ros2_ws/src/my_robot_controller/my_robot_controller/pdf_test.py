from datetime import datetime
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate,
    Table,
    TableStyle,
    Paragraph,
    Image,
    Spacer,
    PageTemplate
)
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.units import inch


# data = [
#     {"idx": 0, "x": 0.879, "y": -0.055, "size": 0.193},
#     {"idx": 1, "x": 1.228, "y": 0.093, "size": 0.284},
#     {"idx": 2, "x": 1.227, "y": -0.48, "size": 0.283},
#     {"idx": 3, "x": 0.802, "y": -0.92, "size": 0.107},
#     {"idx": 4, "x": 0.691, "y": -1.048, "size": 0.13},
#     {"idx": 5, "x": -0.009, "y": -0.841, "size": 0.165},
#     {"idx": 6, "x": -0.34, "y": -1.004, "size": 0.123},
#     {"idx": 7, "x": -0.168, "y": -0.638, "size": 0.081},
#     {"idx": 8, "x": -0.64, "y": -1.069, "size": 0.334},
#     {"idx": 9, "x": -1.054, "y": -0.922, "size": 0.197},
#     {"idx": 10, "x": -1.172, "y": -0.841, "size": 0.227},
#     {"idx": 11, "x": -1.156, "y": -0.315, "size": 0.127},
#     {"idx": 12, "x": -0.791, "y": 0.145, "size": 0.272},
#     {"idx": 13, "x": -0.276, "y": -0.158, "size": 0.097},
#     {"idx": 14, "x": -0.655, "y": -0.143, "size": 0.099},
#     {"idx": 15, "x": -0.116, "y": 0.058, "size": 0.116},
#     {"idx": 16, "x": 0.339, "y": 0.018, "size": 0.239},
# ]

data = [
    ["Pothole", "x position", "y position", "size"],
    [0, 0.926, -0.062, 0.23],
    [1, 1.23, 0.096, 0.275],
    [2, 0.96, -0.319, 0.259],
    [3, 1.222, -0.471, 0.282],
    [4, 0.808, -0.92, 0.097],
    [5, 0.733, -1.06, 0.131],
    [6, -0.01, -0.84, 0.25],
    [7, -0.351, -1.005, 0.118],
    [8, -0.149, -0.616, 0.119],
    [9, -0.64, -1.072, 0.328],
    [10, -1.056, -0.922, 0.198],
    [11, -1.172, -0.842, 0.236],
    [12, -1.197, -0.189, 0.26],
    [13, -0.791, 0.145, 0.259],
    [14, -0.21, -0.024, 0.152],
    [15, -0.695, -0.184, 0.149],
    [16, -0.295, -0.17, 0.134],
    [17, 0.19, -0.006, 0.275],
    [18, 0.345, 0.051, 0.247],
    [19, 0.709, -0.065, 0.078],
]


def header(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica-Bold", 12)
    canvas.drawString(inch, letter[1] - 0.75 * inch, "Robot Programming Assignment")
    canvas.restoreState()


def make_pdf(data):
    # Create a PDF document
    pdf_filename = "output.pdf"
    document = SimpleDocTemplate(pdf_filename, pagesize=letter)

    page_width, page_height = letter
    table_width = page_width - 2 * 72
    elements = []
    document.addPageTemplates([PageTemplate(id="header", frames=[header])])

    title_style = getSampleStyleSheet()["Title"]
    title_text = "Pothole Reporter"
    title = Paragraph(title_text, title_style)
    elements.append(title)

    subheading_style = getSampleStyleSheet()["Heading2"]
    subheading_text = "Your Subheading"
    subheading = Paragraph(subheading_text, subheading_style)
    elements.append(subheading)

    # Create a table
    table = Table(data, colWidths=[table_width / len(data[0])] * len(data[0]))
    style = TableStyle(
        [
            ("BACKGROUND", (0, 0), (-1, 0), colors.grey),
            ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
            ("ALIGN", (0, 0), (-1, -1), "CENTER"),
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("BOTTOMPADDING", (0, 0), (-1, 0), 12),
            ("BACKGROUND", (0, 1), (-1, -1), colors.beige),
        ]
    )

    table.setStyle(style)
    elements.append(table)
    elements.append(Spacer(1, 0.1 * inch))

    # Add a paragraph at the bottom
    paragraph_style = getSampleStyleSheet()["BodyText"]
    paragraph_text = """
    Adding a bunch of text here to see what this ends up looking like and to see of this will still work and stuff.
    """
    paragraph = Paragraph(paragraph_text, paragraph_style)
    elements.append(paragraph)

    elements.append(Spacer(1, 0.1 * inch))

    # Add an image to the elements list
    image_filename = "sample6.png"
    image = Image(image_filename, width=200, height=100)
    elements.append(image)

    # Build the PDF document
    document.build(elements)


def generate_pothole_report(report_data):
    print(report_data)
    now = datetime.now()  # current date and time
    date_time = now.strftime("%d-%M-%Y_%H-%M-%S")

    file_name = f"test_report_{date_time}.pdf"
    document_title = "sample"
    title = "Robot Programming Assignment"
    sub_title = "LIMO Pothole Evaluation"

    # creating a pdf object
    pdf = canvas.Canvas(file_name)
    pdf.setTitle(document_title)
    pdf.drawCentredString(300, 770, title)
    pdf.setFillColorRGB(0, 0, 255)
    pdf.setFont("Courier-Bold", 24)
    pdf.drawCentredString(290, 720, sub_title)

    pdf.line(30, 710, 550, 710)
    text = pdf.beginText(40, 680)
    text.setFont("Courier", 18)
    text.setFillColor(colors.black)
    # for line in report_data:
    pdf.drawText(text)

    # Define the table headers
    table_headers = ["Pothole", "X", "Y", "Size"]
    # Set the starting point for the table
    x, y = 100, 700

    # Extract data from the list of dictionaries
    # table_data = [
    #     [entry["idx"], entry["x"], entry["y"], entry["z"], entry["size"]]
    #     for entry in report_data
    # ]

    # Draw headers
    for header in table_headers:
        pdf.drawString(x, y, header)
        x += 100

    # Draw data
    y -= 20
    for entry in report_data:
        x = 100
        y -= 20
        for key in ["idx", "x", "y", "size"]:
            pdf.drawString(x, y, str(entry[key]))
            x += 100

    # Create the table
    # table = Table([table_headers] + table_data)
    #
    # # Add style to the table
    # style = TableStyle(
    #     [
    #         ("BACKGROUND", (0, 0), (-1, 0), colors.grey),
    #         ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
    #         ("ALIGN", (0, 0), (-1, -1), "CENTER"),
    #         ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
    #         ("BOTTOMPADDING", (0, 0), (-1, 0), 12),
    #         ("BACKGROUND", (0, 1), (-1, -1), colors.beige),
    #         ("GRID", (0, 0), (-1, -1), 1, colors.black),
    #     ]
    # )
    #
    # table.setStyle(style)

    # Build the PDF document
    # pdf.build([table])

    # pdf.drawInlineImage(image, 130, 400)
    pdf.save()
    #     for idx, marker in enumerate(self.markers):


#         self.get_logger().info(f"{idx}: {marker}")
make_pdf(data)
# generate_pothole_report(data)

# file_name = "sample.pdf"
# document_title = "sample"
# title = "Robot Programming Assignment"
# sub_title = "LIMO Pothole Evaluation"
# text_lines = [
#     "Technology makes us aware of",
#     "the world around us.",
# ]
# image = "../resources/simple_pothole_world.jpg"
#
#
# # creating a pdf object
# pdf = canvas.Canvas(file_name)
#
# # setting the title of the document
# pdf.setTitle(document_title)
#
#
# # registering a external font in python
# # pdfmetrics.registerFont(TTFont("abc", "SakBunderan.ttf"))
#
# # creating the title by setting it's font
# # and putting it on the canvas
# # pdf.setFont("abc", 36)
# pdf.drawCentredString(300, 770, title)
#
#
# # creating the subtitle by setting it's font,
# # colour and putting it on the canvas
# pdf.setFillColorRGB(0, 0, 255)
# pdf.setFont("Courier-Bold", 24)
# pdf.drawCentredString(290, 720, sub_title)
#
#
# # drawing a line
# pdf.line(30, 710, 550, 710)
#
# # creating a multiline text using
# # textline and for loop
# text = pdf.beginText(40, 680)
# text.setFont("Courier", 18)
# text.setFillColor(colors.black)
# for line in text_lines:
#     text.textLine(line)
# pdf.drawText(text)
#
# # drawing a image at the
# # # specified (x.y) position
# pdf.drawInlineImage(image, 130, 400)
#
# # saving the pdf
# pdf.save()
